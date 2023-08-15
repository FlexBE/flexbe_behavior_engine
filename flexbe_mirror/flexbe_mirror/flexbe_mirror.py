# Copyright 2023 Philipp Schillinger, Team ViGIR, Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger, Team ViGIR, Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Class to handle the FlexBE mirror of onboard statemachine."""

try:
    from prctl import set_name as set_thread_name
except Exception:
    def set_thread_name(name):
        print("Python thread names are not visible in ps/top unless you install prctl")

import threading
import time
import traceback

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import Empty, String, Int32, UInt32

from flexbe_core import Logger, MIN_UI_VERSION
from flexbe_core.core import LockableStateMachine, OperatableStateMachine
from flexbe_core.core import PreemptableState, PreemptableStateMachine, StateMap
from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_msgs.msg import ContainerStructure, BehaviorSync, BEStatus

from .mirror_state import MirrorState
from .mirror_state_machine import MirrorStateMachine
from .mirror_concurrency_container import MirrorConcurrencyContainer


class FlexbeMirror(Node):
    """Class to handle the FlexBE mirror of onboard statemachine."""

    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('flexbe_mirror')

        self._sm = None
        ProxyPublisher.initialize(self)
        ProxySubscriberCached.initialize(self)
        Logger.initialize(self)

        MirrorState.initialize_ros(self)
        PreemptableState.initialize_ros(self)
        PreemptableStateMachine.initialize_ros(self)
        LockableStateMachine.initialize_ros(self)

        self._timing_event = threading.Event()  # Used for wait timer

        self._running = False
        self._stopping = False
        self._active_id = BehaviorSync.INVALID
        self._starting_path = None
        self._current_struct = None
        self._struct_buffer = []
        self._sync_lock = threading.Lock()
        self._state_map = None
        self._system_clock = Clock()
        self._active_thread_start = None

        # set up proxys for sm <--> GUI communication
        # publish topics
        self._heartbeat_pub = self.create_publisher(Int32, Topics._MIRROR_HEARTBEAT_TOPIC, 2)
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._version_sub = self.create_subscription(String, Topics._UI_VERSION_TOPIC,
                                                     self._version_callback, qos_profile=latching_qos)
        self._request_struct_pub = self.create_publisher(Int32, Topics._REQUEST_STRUCTURE_TOPIC, 2)

        # listen for mirror control messages using standard subscriptions
        self._status_sub = self.create_subscription(BEStatus, Topics._ONBOARD_STATUS_TOPIC, self._status_callback, 10)
        self._struct_sub = self.create_subscription(ContainerStructure, Topics._MIRROR_STRUCTURE_TOPIC,
                                                    self._mirror_structure_callback, 10)
        self._sync_sub = self.create_subscription(BehaviorSync, Topics._MIRROR_SYNC_TOPIC, self._sync_callback, 10)
        self._preempt_sub = self.create_subscription(Empty, Topics._MIRROR_PREEMPT_TOPIC, self._preempt_callback, 10)
        self._onboard_heartbeat_sub = self.create_subscription(BehaviorSync, Topics._ONBOARD_HEARTBEAT_TOPIC,
                                                               self._onboard_heartbeat_callback, 10)
        self._sync_heartbeat_mismatch_counter = 0

        # Use proxy publisher/subscriber for access in states
        # but just initialize here once for all
        self._beh_update_pub = ProxyPublisher({Topics._BEHAVIOR_UPDATE_TOPIC: String})

        self._outcome_sub = ProxySubscriberCached()
        self._outcome_sub.subscribe(Topics._OUTCOME_TOPIC, UInt32, inst_id=id(self))
        self._outcome_sub.enable_buffer(Topics._OUTCOME_TOPIC)

        # no clean way to wait for publisher to be ready...
        Logger.loginfo('--> Mirror - setting up publishers and subscribers ...')
        threading.Event().wait(1.0)  # Give publishers time to initialize

        # Require periodic events in case behavior is not connected to allow orderly shutdown
        self._heartbeat_timer = self.create_timer(2.0, self.heartbeat_timer_callback)

        Logger.loginfo('--> Mirror - ready!')

    def _version_callback(self, msg):
        vui = FlexbeMirror._parse_version(msg.data)
        vex = FlexbeMirror._parse_version(MIN_UI_VERSION)
        if vui < vex:
            Logger.logwarn('FlexBE App needs to be updated!\n'
                           f'Mirror requires at least version {MIN_UI_VERSION}, '
                           f' but you have {msg.data}\n'
                           'Please update the flexbe_app software.')

    @staticmethod
    def _parse_version(v):
        result = 0
        offset = 1
        for n in reversed(v.split('.')):
            result += int(n) * offset
            offset *= 100
        return result

    def get_elapsed_str(self, start_time):
        """Return a truncated time string for debugging."""
        elapsed = self._system_clock.now() - start_time
        sec, nsec = start_time.seconds_nanoseconds()
        return f"started at {sec & 0xFFFF}.{nsec//1000:06d} s (elapsed={elapsed.nanoseconds/1e9} s)"

    def heartbeat_timer_callback(self):
        """
        Allow monitoring of Mirror liveness.

        Use negative time in seconds if mirror is not active, and loop count when running

        Guarantee some event triggers wake up so that we can catch Ctrl-C in case where no active messages are available.
        """
        heartbeat = Int32(data=-(self.get_clock().now().seconds_nanoseconds()[0] & 0x0000FFFF))
        if self._sm is not None and self._running:
            heartbeat.data = self._sm._total_loop_count

        self._heartbeat_pub.publish(heartbeat)

    def shutdown_mirror(self):
        """Shut mirror down."""
        try:
            print(f"    Shutting down behavior mirror '{self._active_id}' ...", flush=True)
            with self._sync_lock:
                self._stopping = True
                if self._sm is not None and self._running:
                    print(f"    Mirror '{self._active_id}' is shutting down with behavior still active!", flush=True)
                    self._wait_stop_running(self._system_clock.now())

                    if self._running:
                        print(f"    Failed to stop mirror '{self._active_id}' while it is already running!", flush=True)
                        return False
                    else:
                        self._stopping = False
                        self._active_id = BehaviorSync.INVALID
                        self._sm = None
                        self._current_struct = None

                print('    Stop heartbeat timer ...', flush=True)
                self.destroy_timer(self._heartbeat_timer)
                threading.Event().wait(0.05)

                print('   Remove subscribers ...', flush=True)
                self._outcome_sub.unsubscribe_topic(Topics._OUTCOME_TOPIC, inst_id=id(self))

                print('   Remove publishers ...', flush=True)
                self._beh_update_pub.remove_publisher(Topics._BEHAVIOR_UPDATE_TOPIC)

                print('    Mirror is shutdown!', flush=True)
                return True
            return False  # No active behavior

        except Exception as exc:  # pylint: disable=W0703
            print(f"Exception shutting down behavior mirror {type(exc)}\n   {exc}", flush=True)
            import traceback
            print(traceback.format_exc().replace("%", "%%"), flush=True)

    def _mirror_structure_callback(self, msg):
        start_time = self._system_clock.now()
        Logger.loginfo(f'--> Mirror - received updated structure with checksum id = {msg.behavior_id}')
        Logger.localinfo(f'   at {start_time.nanoseconds} ns')
        thread = threading.Thread(target=self._activate_mirror, args=[msg, start_time],
                                  name=f"activate_mirror_{msg.behavior_id}_{start_time.nanoseconds}")
        thread.daemon = True
        thread.start()

    def _activate_mirror(self, struct_msg, start_time):
        set_thread_name("act" + f"{start_time.nanoseconds}"[-12:])  # only 15 chars allowed

        with self._sync_lock:
            Logger.loginfo(f'Activate mirror for behavior id = {struct_msg.behavior_id} ...')
            self._wait_stopping(start_time)

            if self._running:
                Logger.localwarn(f'Received a new mirror structure for checksum id={struct_msg.behavior_id} '
                                 f'while mirror is already running with active id={self._active_id}; '
                                 'adding to buffer for later!')
                self._struct_buffer.append(struct_msg)
                return
            elif self._active_id not in (BehaviorSync.INVALID, struct_msg.behavior_id):
                Logger.localwarn(f'Received mirror structure id={struct_msg.behavior_id} that does '
                                 f'not match active id = {self._active_id} - will ignore!')
                return

            # At this point, either active_id is invalid or same behavior checksum id
            Logger.localinfo(f'Process the updated mirror structure for checksum id = {struct_msg.behavior_id} ...')
            self._active_id = struct_msg.behavior_id  # in case invalid
            self._struct_buffer = []
            self._mirror_state_machine(struct_msg)
            if self._sm:
                Logger.localinfo(f'Mirror built for behavior id = {self._active_id}.')
                self._sm.set_name("root")
            else:
                Logger.localwarn(f'Error processing mirror structure for behavior checksum id = {self._active_id} ...')
                Logger.logwarn('Requesting a new mirror structure from onboard ...')
                self._request_struct_pub.publish(Int32(data=struct_msg.behavior_id))
                self._active_id = struct_msg.behavior_id
                return

            self._running = True  # Ready to start execution, so flag it as so before releasing sync

        # Release sync lock and execute the mirror
        Logger.localinfo(f"Mirror - begin execution of active mirror behavior id = '{self._sm.id}' "
                         f"{self.get_elapsed_str(start_time)}")

        try:
            self._execute_mirror(start_time)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f'Exception in activate mirror: {type(exc)} started at {start_time.nanoseconds} ns ...\n  {exc}')
            Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")
            self._running = False  # normally set false in execute_mirror (but not if exeception)

        Logger.localwarn(f"Done executing mirror {self._active_id} from activation "
                         f"{self.get_elapsed_str(start_time)}")

    def _status_callback(self, msg):
        start_time = self._system_clock.now()
        if msg.code == BEStatus.STARTED:
            Logger.localinfo(f'Mirror - received BEstate={msg.code} - start mirror')
            #                      f'start mirror with behavior id = {msg.behavior_id} started at {start_time.nanoseconds} ns')
            thread = threading.Thread(target=self._start_mirror, args=[msg, start_time],
                                      name=f"start_mirror_{msg.behavior_id}_{start_time.nanoseconds}")
            thread.daemon = True
            thread.start()
        elif self._sm and not self._stopping and msg.code == BEStatus.FINISHED:
            Logger.localinfo(f'Mirror - received BEstate={msg.code} - stop current mirror')
            thread = threading.Thread(target=self._stop_mirror, args=[msg, start_time],
                                      name=f"stop_mirror_{msg.behavior_id}_{start_time.nanoseconds}")
            thread.daemon = True
            thread.start()

    def _start_mirror(self, msg, start_time):
        set_thread_name("str" + f"{start_time.nanoseconds}"[-12:])  # only 15 chars allowed
        with self._sync_lock:
            self._wait_stopping(start_time)

            if self._running:
                if self._active_id != msg.behavior_id:
                    Logger.loginfo(f"Tried to start mirror for id={msg.behavior_id} while"
                                   f" mirror for id={self._active_id} is already running - will ignore start request!")
                    Logger.localwarn(f" (active thread started {self._active_thread_start}) "
                                     f" {self.get_elapsed_str(start_time)}")
                else:
                    Logger.localinfo(f" Received start request for already active {self._active_id} "
                                     f"(active thread started {self._active_thread_start}) "
                                     f" {self.get_elapsed_str(start_time)}")
                return

            Logger.localinfo(f" Start request mirror for {msg.behavior_id} in thread {self.get_elapsed_str(start_time)}")

            if len(msg.args) > 0:
                self._starting_path = "/" + msg.args[0][1:].replace("/", "_mirror/") + "_mirror"

            self._active_id = msg.behavior_id
            assert self._sm is None, ("No SM should be active with start command here "
                                      f"(id = {self._active_id}, {self._sm.id}) {self.get_elapsed_str(start_time)}")
            if len(self._struct_buffer) > 0:
                # Logger.localinfo(f"Building mirror structure for checksum id={msg.behavior_id} "
                #                  f"current len(struct_buffer)={len(self._struct_buffer)} ...")
                while self._sm is None and len(self._struct_buffer) > 0:
                    struct = self._struct_buffer[0]
                    self._struct_buffer = self._struct_buffer[1:]
                    if struct.behavior_id == self._active_id:
                        self._mirror_state_machine(struct)
                        Logger.localinfo(f"Mirror built for checksum '{self._active_id}'")
                    else:
                        Logger.logwarn(f"Discarded mismatching buffered structure for checksum '{struct.behavior_id}'")

            if self._sm is None:
                Logger.localwarn(f"Missing correct mirror structure for starting behavior checksum id ='{msg.behavior_id}'")
                Logger.localwarn(f"   Canceling start request for behavior checksum id ={msg.behavior_id} "
                                 f"{self.get_elapsed_str(start_time)}")
                Logger.logwarn('Requesting mirror structure from onboard ...')
                self._request_struct_pub.publish(Int32(data=msg.behavior_id))
                self._active_id = msg.behavior_id
                return

            if self._sm._current_state is None:
                try:
                    # Set default initial state for state machine
                    self._sm._current_state = self._state_map[struct.containers[0].state_id]
                    Logger.localwarn(f'Set initial state as {self._sm._current_state.path} for checksum id ={msg.behavior_id}')
                except Exception:  # pylint: disable=W0703
                    Logger.localwarn(f'Failed to set the initial state for checksum id ={msg.behavior_id}')

            self._running = True  # Ready to execute, so flag as running before releasing sync lock

        try:
            Logger.localinfo(f"Begin mirror execution for checksum '{self._active_id}' "
                             f"{self.get_elapsed_str(start_time)}")
            self._execute_mirror(start_time)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f'Exception in start_mirror: {type(exc)} ...\n  {exc}')
            Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")
            self._running = False  # normally set false in execute_mirror (but not if exception)

        Logger.localwarn(f"Mirror execution for '{self._active_id}' is finished "
                         f"{self.get_elapsed_str(start_time)}")

    def _stop_mirror(self, msg, start_time):
        # self.get_logger().info('--> Mirror - request to stop mirror for '
        #                        f'checksum id={msg.behavior_id if isinstance(msg, BEStatus) else None} '
        #                        f'- waiting for sync lock ...')

        set_thread_name("stp" + f"{start_time.nanoseconds}"[-12:])  # only 15 chars allowed
        with self._sync_lock:
            Logger.localinfo(f"Mirror '{self._active_id}' - stopping mirror "
                             f"for checksum id={msg.behavior_id if msg is not None else 'None'} "
                             f" {self.get_elapsed_str(start_time)}")
            self._stopping = True
            if self._sm is not None and self._running:
                if msg is None:
                    Logger.logwarn('Onboard behavior stop request (from sync)!')
                elif msg.code == BEStatus.FINISHED:
                    Logger.loginfo('Onboard behavior finished successfully.')
                    self._beh_update_pub.publish(Topics._BEHAVIOR_UPDATE_TOPIC, String())
                elif msg.code == BEStatus.SWITCHING:
                    self._starting_path = None
                    Logger.loginfo('Onboard performing behavior switch.')
                elif msg.code == BEStatus.READY:
                    Logger.loginfo('Onboard engine just started, stopping currently running mirror.')
                    self._beh_update_pub.publish(Topics._BEHAVIOR_UPDATE_TOPIC, String())
                else:
                    Logger.logwarn('Onboard behavior failed!')
                    self._beh_update_pub.publish(Topics._BEHAVIOR_UPDATE_TOPIC, String())

                self._wait_stop_running(start_time)

                self._sm.destroy()

            else:
                Logger.localinfo('Stop request - but no onboard behavior is currently active.')

            self._active_id = BehaviorSync.INVALID
            self._sm = None
            self._current_struct = None
            self._outcome_sub.remove_last_msg(Topics._OUTCOME_TOPIC, clear_buffer=True)

            if msg is not None and msg.code != BEStatus.SWITCHING:
                Logger.loginfo('\033[92m--- Behavior Mirror ready! ---\033[0m')

            # Logger.localinfo("Mirror - stopped mirror for "
            #                  f"behavior id={msg.behavior_id if msg is not None else BehaviorSync.INVALID} "
            #                  f"- ready to release sync lock in stopping thread {self.get_elapsed_str(start_time)} s ...")
            self._stopping = False

    def _sync_callback(self, msg):
        start_time = self._system_clock.now()
        if msg.behavior_id == self._active_id:
            Logger.logwarn(f"--> Mirror - sync request for behavior id={msg.behavior_id} - restart mirror")
            thread = threading.Thread(target=self._restart_mirror, args=[msg, start_time])
            thread.daemon = True
            thread.start()
        else:
            Logger.error('--> Mirror - Cannot synchronize!\n    Different behavior is running onboard, '
                         'please stop execution while we reset the mirror!')
            Logger.localwarn(f"Cannot synchronize!  onboard checksum id={msg.behavior_id} active={self._active_id} "
                             f"{self.get_elapsed_str(start_time)}")
            thread = threading.Thread(target=self._stop_mirror, args=[None, start_time])
            thread.daemon = True
            thread.start()

    def _onboard_heartbeat_callback(self, msg):
        try:
            """Use heartbeat to monitor for persistent sync issues."""
            if self._active_id == BehaviorSync.INVALID:
                return  # do not check sync status if no behavior is active

            if msg.behavior_id == self._active_id:
                if self._sm is not None:
                    mirror_status = self._sm.get_latest_status()
                    if mirror_status.behavior_id != self._active_id:
                        Logger.localwarn(f"mirror_status.behavior_id ({mirror_status.behavior_id}) != "
                                         f" self._active_id ({self._active_id})")

                    mirror_status.behavior_id = self._active_id

                    if msg.current_state_checksums != mirror_status.current_state_checksums:
                        if self._sync_heartbeat_mismatch_counter > 0:
                            # Two consecutive out of sync heartbeats
                            onboard_state_path = "Unknown"
                            if len(msg.current_state_checksums) > 0:
                                # Use deepest state as the best path estimate
                                ob_state_id, ob_out = StateMap.unhash(msg.current_state_checksums[-1])
                                ob_state = self._state_map[ob_state_id]
                                onboard_state_path = ob_state.path

                        if self._sync_heartbeat_mismatch_counter % 5 == 1:
                            Logger.error(f'OCS is possibly out of sync - onboard state is {onboard_state_path}\n'
                                         f'    Check UI and consider manual re-sync!\n'
                                         '    (mismatch may be temporarily understandable for rapidly changing outcomes)'
                                         f' {self._sync_heartbeat_mismatch_counter}')
                            Logger.localinfo(f'IDs {msg.behavior_id} {self._active_id} : \n'
                                             f'   Onboard IDs: {msg.current_state_checksums}\n    '
                                             f'Mirror IDs {mirror_status.current_state_checksums}')

                            for state_hash in msg.current_state_checksums:
                                try:
                                    ob_state_id, ob_out = StateMap.unhash(state_hash)
                                    ob_state = self._state_map[ob_state_id]
                                    Logger.localinfo(f"  onboard {ob_state_id} : '{ob_state.name}' "
                                                     f"out={ob_out} - {ob_state.path}")
                                except Exception as exc:  # pylint: disable=W0703
                                    Logger.localinfo(f" error for onboard state hash {state_hash} - {type(exc)} - {exc}")
                            for state_hash in mirror_status.current_state_checksums:
                                try:
                                    mr_state_id, mr_out = StateMap.unhash(state_hash)
                                    mr_state = self._state_map[mr_state_id]
                                    Logger.localinfo(f"  mirror {mr_state_id} : '{mr_state.name.replace('_mirror', '')}' "
                                                     f" out={mr_out} - {mr_state.path.replace('_mirror', '')}")
                                except Exception as exc:  # pylint: disable=W0703
                                    Logger.localinfo(f" error for mirror state hash {state_hash} - {type(exc)} - {exc}")
                            Logger.localinfo(30 * "=")

                        else:
                            # Start counting mismatches
                            self._sync_heartbeat_mismatch_counter = 1
                    else:
                        # Reset mismatch counter
                        self._sync_heartbeat_mismatch_counter = 0
                elif self._active_id != 0:
                    Logger.warning(f'Received matching behavior id {msg.behavior_id} with no mirror state machine active!')
                else:
                    Logger.warning(f'Received invalid behavior id {msg.behavior_id} with active id = {self._active_id} active!')

            elif msg.INVALID not in (msg.behavior_id, self._active_id):
                if self._sync_heartbeat_mismatch_counter % 10 == 1:
                    Logger.error('Out of sync! Different behavior is running onboard, please stop execution! '
                                 f'{self._sync_heartbeat_mismatch_counter}')
                self._sync_heartbeat_mismatch_counter += 1
            elif not self._stopping:
                self._sync_heartbeat_mismatch_counter += 1
                if self._sync_heartbeat_mismatch_counter % 10 == 1:
                    Logger.warning(f"Mismatched behavior ids ('{msg.behavior_id}', '{self._active_id}')- "
                                   f"please restart behavior! {self._sync_heartbeat_mismatch_counter}")
            else:
                Logger.localinfo(f"Heartbeat: mirror is stopping - waiting for  {self._active_id} to stop ...")

        except Exception as exc:
            Logger.localinfo(f"Exception in heartbeat callback {type(exc)} - {exc}")
            Logger.localinfo(f"{traceback.format_exc().replace('%', '%%')}")

    def _wait_stop_running(self, start_time):
        if self._running:
            PreemptableState.preempt = True
            running_cnt = 0
            timing_event = threading.Event()
            while self._running:
                if running_cnt % 2000 == 0:
                    try:
                        Logger.localinfo(f"Waiting for another mirror (start thread {self._active_thread_start}) to stop "
                                         f"with id = '{self._active_id}' "
                                         f"(this {self.get_elapsed_str(start_time)}) (running cnt={running_cnt}) "
                                         f"preempt={PreemptableState.preempt}")
                    except Exception:  # pylint: disable=W703
                        # Likely during shutdown
                        print(f"Waiting for another mirror (start thread {self._active_thread_start}) to stop "
                              f"with id = '{self._active_id}' "
                              f"(this {self.get_elapsed_str(start_time)}) (running cnt={running_cnt})", flush=True)
                running_cnt += 1
                if running_cnt > 100000:
                    Logger.logerr(f"Timeout waiting for another mirror ({self._active_thread_start}) to stop running "
                                  f" with {self._active_id} (this {self.get_elapsed_str(start_time)})")
                    return

                timing_event.wait(0.00002)  # Use system time for polling check, never sim_time
            Logger.localinfo(f"Mirror for active id {self._active_id} stopped running (start thread {self._active_thread_start}) "
                             f" ({running_cnt}) (this {self.get_elapsed_str(start_time)})")
            Logger.loginfo("Mirror stopped running!")

    def _wait_stopping(self, start_time):
        if self._stopping:
            stopping_cnt = 0
            timing_event = threading.Event()
            while self._stopping:
                if stopping_cnt % 5000 == 0:
                    try:
                        Logger.localinfo(f"Waiting for another mirror (start thread {self._active_thread_start}) "
                                         f" to finish stopping  with id='{self._active_id}' "
                                         f"(this {self.get_elapsed_str(start_time)}) (stopping {stopping_cnt})... ")
                    except Exception:  # pylint: disable=W0703
                        print(f"Waiting for another mirror (start thread {self._active_thread_start}) "
                              f" to finish stopping  with id='{self._active_id}' "
                              f"(this {self.get_elapsed_str(start_time)}) (stopping {stopping_cnt})... ", flush=True)
                stopping_cnt += 1
                if stopping_cnt > 100000:
                    Logger.logerr(f"Timeout waiting for another mirror to finish stopping with '{self._active_id}'"
                                  f" (this {self.get_elapsed_str(start_time)})")
                    return

                timing_event.wait(0.00002)  # use wall clock not sim time
            Logger.localinfo(f"Mirror completed stopping for active id {self._active_id} "
                             f" (this {self.get_elapsed_str(start_time)}) ({stopping_cnt})!")
            Logger.loginfo("Mirror stopped.")

    def _reinitialize_state_machine(self, state_machine):
        """Reinitialize existing SM when restarting mirror during sync (faster than rebuilding current structure)."""
        state_machine._entering = False
        state_machine._current_state = None
        for state in state_machine._states:
            state._entering = True
            if isinstance(state, MirrorStateMachine):
                self._reinitialize_state_machine(state)

    def _restart_mirror(self, msg, restart_time):
        set_thread_name("rsm" + f"{restart_time.nanoseconds}"[-12:])  # only 15 chars allowed
        with self._sync_lock:
            if self._sm is not None and self._running:
                self._wait_stop_running(restart_time)

            Logger.localinfo(f"Restarting mirror for synchronization of behavior checksum id ={msg.behavior_id} "
                             f"in thread {self.get_elapsed_str(restart_time)} ...")

            # Clear existing outcome messages
            self._outcome_sub.remove_last_msg(Topics._OUTCOME_TOPIC, clear_buffer=True)
            MirrorState._last_state_id = None
            MirrorState._last_state_outcome = None

            try:
                self._starting_path = None
                if self._sm is not None and self._sm.id == msg.behavior_id:
                    self._active_id = msg.behavior_id
                    start = time.time()
                    PreemptableState.preempt = False  # Reset preempt flag before restarting
                    self._reinitialize_state_machine(self._sm)
                    end = time.time()
                    Logger.localinfo(f"Done reinitializing the existing state machine with matching "
                                     f"behavior id='{msg.behavior_id}' in thread "
                                     f"{self.get_elapsed_str(restart_time)} (reinitialized in {end-start} seconds) ")
                else:
                    # Reconstruct the state machine from structure
                    self._mirror_state_machine(self._current_struct)
                    if self._sm is None:
                        Logger.localwarn(f"Missing correct mirror structure for restarting "
                                         f"behavior checksum id ={msg.behavior_id}\n"
                                         f"    Canceling restart request for behavior checksum id ={msg.behavior_id} "
                                         f"in thread {self.get_elapsed_str(restart_time)} ")
                        Logger.logwarn('Requesting mirror structure from onboard ...')
                        self._pub.publish(Topics._REQUEST_STRUCTURE_TOPIC, Int32(data=msg.behavior_id))
                        self._active_id = msg.behavior_id
                        return

                Logger.localinfo(f' Reset active states using current state checksums: {msg.current_state_checksums}')
                for active_state in msg.current_state_checksums:
                    # For now, just set the active state to the lowest order state with no outcome
                    state_id, outcome = StateMap.unhash(active_state)
                    state = self._state_map[state_id]
                    if state is not None:
                        if self._starting_path is None:
                            self._starting_path = state.path

                        if outcome != 0 and MirrorState._last_state_id is None:
                            # Some outcome to process
                            MirrorState._last_state_id = state_id
                            MirrorState._last_state_outcome = outcome

                        state._entering = False  # state considered already active with sync
                        parent = state.parent
                        while parent is not None:
                            parent._entering = False  # state considered already active
                            if isinstance(parent, MirrorConcurrencyContainer):
                                if parent._current_state is None:
                                    parent._current_state = []
                                if state not in parent._current_state:
                                    parent._current_state.append(state)
                            elif isinstance(parent, MirrorStateMachine):
                                parent._current_state = state
                            else:
                                Logger.logerr(f"            Sync: Unexpected parent reference {parent.name} ({type(parent)}) "
                                              f"from {state.name} in {self._sm.name}")
                            state = parent
                            parent = parent.parent
                    else:
                        Logger.logerr(f"        Unknown state from {state_id} in {self._sm.name}  from restart "
                                      f"in thread {self.get_elapsed_str(restart_time)}!")
                curst = self._sm._current_state
                self._sm._last_deep_states_list = self._sm.get_deep_states()
                Logger.localwarn(f" Restart SM with current top-level state = {curst.name if curst is not None else 'None'} "
                                 f"starting path={self._starting_path}")
                Logger.localinfo(f"     active states = {self._sm.get_latest_status()}")
                self._running = True  # set running while we have sync lock
            except (AttributeError, RuntimeError) as exc:
                Logger.loginfo(f'Stopping synchronization because behavior{msg.behavior_id} has stopped.')
                Logger.localinfo(f"{type(exc)} - {exc}")
                return

        try:
            Logger.localinfo("Execute mirror after sync lock of restart mirror"
                             f" for synchronization of behavior '{msg.behavior_id}' from restart "
                             f"in thread {self.get_elapsed_str(restart_time)} ...")
            self._execute_mirror(restart_time)
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f'Exception in restart_mirror in thread {self.get_elapsed_str(restart_time)}:\n    {type(exc)} - {exc}')
            Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")
            self._running = False

        Logger.localwarn(f"Finished execution of restart request for behavior checksum id ={msg.behavior_id} "
                         f"in thread {self.get_elapsed_str(restart_time)}.")

    def _execute_mirror(self, start_time):
        if self._active_thread_start is not None:
            Logger.localwarn(f"Trying to start execution for {start_time.nanoseconds} but "
                             f"older {self._active_thread_start} is still active!")
            Logger.logerr("Mirror issue - shutdown all threads - need to reattach!")
            PreemptableState.preempt = True
            return

        self._running = True
        self._active_thread_start = start_time.nanoseconds  # Track starting time
        Logger.loginfo("Executing mirror ...")
        Logger.localinfo(f"  in thread {self.get_elapsed_str(start_time)} s ")
        if self._starting_path is not None:
            LockableStateMachine.path_for_switch = self._starting_path
            Logger.loginfo("Starting mirror in state " + self._starting_path)
            self._starting_path = None

        result = PreemptableStateMachine._preempted_name
        try:
            result = self._sm.spin(start_time)
            Logger.localinfo(f"Mirror finished spin with result '{result}' after {self.get_elapsed_str(start_time)} s")
            self._sm.destroy()
        except Exception as exc:
            try:
                Logger.logerr('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc))
                Logger.localerr(traceback.format_exc().replace("%", "%%"))
            except Exception:  # pylint: disable=W0703
                # Likely the loggers are dead if we ctrl-C'd during active behavior
                # so just try a simple print
                print('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc), flush=True)
                print(traceback.format_exc().replace("%", "%%"), flush=True)
            result = PreemptableStateMachine._preempted_name

        self._active_thread_start = None
        self._running = False

    def _mirror_state_machine(self, msg):
        start = time.time()  # Track construction time
        try:
            self._current_struct = msg
            self._state_map = StateMap()
            root = None
            for con_msg in msg.containers:
                if con_msg.path.find('/') == -1:
                    root = con_msg.path
                    break

            # self.get_logger().info(f"Constructing top-level mirror for {root} ...")
            self._add_node(msg, root)

            if self._sm:
                self._sm.id = msg.behavior_id
                # verify checksums of all states
                # self.get_logger().info(f"Validate checksums for all states in '{root}' ...")
                for con_msg in msg.containers:
                    if con_msg.path.find('/') != -1:
                        hash_val = self._state_map.get_path_hash(con_msg.path)
                        if hash_val:
                            state = self._state_map[hash_val]
                            for path_seg in con_msg.path.split("/"):
                                assert path_seg in state.path, (f"Mismatched state id={state.state_id} for {state.path} vs. "
                                                                f"({hash_val}) {con_msg.path}")
                end = time.time()
                Logger.localinfo(f"Constructed mirror for behavior id ='{self._sm.id}' in {end - start} seconds !")
                return  # success here
            else:
                Logger.logerr(f"Failed to construct mirror SM for {root}!")

        except Exception as exc:
            self.get_logger().warn(f"_mirror_statemachine Exception: {type(exc)} - {exc}")
            self.get_logger().warn(f"{traceback.format_exc().replace('%', '%%')}")
            self._sm = None

        end = time.time()
        Logger.localinfo(f"Failed to construct mirror of state machine '{msg.behavior_id}' in {end - start} seconds !")

    def _add_node(self, msg, path):
        container = None
        for con_msg in msg.containers:
            if con_msg.path == path:
                container = con_msg
                break

        transitions = None
        if container.transitions is not None:
            transitions = {}
            for i in range(len(container.transitions)):
                transitions[container.outcomes[i]] = container.transitions[i] + '_mirror'

        path_frags = path.split('/')
        container_name = path_frags[len(path_frags) - 1]

        if len(container.children) > 0:
            sm_outcomes = []
            for outcome in container.outcomes:
                sm_outcomes.append(outcome + '_mirror')
            if container.type == OperatableStateMachine.ContainerType.ConcurrencyContainer.value:
                sm = MirrorConcurrencyContainer(container_name, path, outcomes=sm_outcomes)
            else:
                sm = MirrorStateMachine(container_name, path, outcomes=sm_outcomes)

            self._state_map.add_state(path, sm)
            assert sm.state_id == container.state_id, ("Failed to validate container state_id "
                                                       f"= {sm.state_id} vs. {container.state_id}")

            with sm:
                for child in container.children:
                    self._add_node(msg, path + '/' + child)
            if len(transitions) > 0:
                container_transitions = {}
                for i in range(len(container.transitions)):
                    container_transitions[sm_outcomes[i]] = transitions[container.outcomes[i]]
                MirrorStateMachine.add(container_name + '_mirror', sm, transitions=container_transitions)
            else:
                # Add instance attributes to top-level state machine
                sm._state_map = self._state_map
                sm._total_loop_count = 0

                self._sm = sm

        else:
            # Basic state
            # self.get_logger().info(f"  Adding mirror state {container_name} at {path} ...")
            assert container.type == 0, f"{container_name} - Non-containers should have type 0 not {container.type}!"
            mrst = MirrorState(container_name, path, container.outcomes, container.autonomy)
            self._state_map.add_state(path, mrst)
            assert mrst.state_id == container.state_id, ("Failed to validate container state_id "
                                                         f"= {mrst.state_id} vs. {container.state_id}")
            MirrorStateMachine.add(container_name + '_mirror', mrst, transitions=transitions)

    def _preempt_callback(self, msg):
        # pylint: disable=unused-argument
        if self._sm is not None:
            Logger.logwarn('Explicit preempting is currently ignored, mirror should be preempted by onboard behavior.')
        else:
            Logger.logwarn('Could not preempt mirror because it seems not to be running!')
