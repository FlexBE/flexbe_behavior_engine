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

import threading
import traceback
import zlib


from rclpy.node import Node
from std_msgs.msg import Empty, String, Int32, UInt8

from flexbe_core import Logger
from flexbe_core.core import PreemptableState, PreemptableStateMachine, LockableStateMachine
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_msgs.msg import ContainerStructure, BehaviorSync, BEStatus

from .mirror_state import MirrorState
from .mirror_state_machine import MirrorStateMachine


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

        # set up proxys for sm <--> GUI communication
        # publish topics
        self._pub = ProxyPublisher({'flexbe/behavior_update': String,
                                    'flexbe/request_mirror_structure': Int32})
        self._heartbeat_pub = self.create_publisher(Int32, 'flexbe/mirror/heartbeat', 2)

        self._timing_event = threading.Event()  # Used for wait timer
        self._running = False
        self._stopping = False
        self._active_id = BehaviorSync.INVALID
        self._starting_path = None
        self._current_struct = None
        self._struct_buffer = []
        self._sync_lock = threading.Lock()
        self._state_checksums = {}

        self._outcome_topic = 'flexbe/mirror/outcome'

        # listen for mirror message
        self._sub = ProxySubscriberCached()
        self._sub.subscribe(self._outcome_topic, UInt8, inst_id=id(self))
        self._sub.enable_buffer(self._outcome_topic)

        self._sub.subscribe('flexbe/status', BEStatus, self._status_callback, inst_id=id(self))
        self._sub.subscribe('flexbe/mirror/structure', ContainerStructure, self._mirror_structure_callback, inst_id=id(self))
        self._sub.subscribe('flexbe/mirror/sync', BehaviorSync, self._sync_callback, inst_id=id(self))
        self._sub.subscribe('flexbe/mirror/preempt', Empty, self._preempt_callback, inst_id=id(self))
        self._sub.subscribe('flexbe/heartbeat', BehaviorSync, self._heartbeat_callback, inst_id=id(self))
        self._sync_heartbeat_mismatch_counter = 0

        # no clean way to wait for publisher to be ready...
        Logger.loginfo('--> Mirror - setting up publishers and subscribers ...')
        self._timing_event.wait(1.0)  # Give publishers time to initialize

        # Require periodic events in case behavior is not connected to allow orderly shutdown
        self._heartbeat_timer = self.create_timer(2.0, self.heartbeat_timer_callback)

        Logger.loginfo('--> Mirror - ready!')

    def heartbeat_timer_callback(self):
        """
        Allow monitoring of Mirror liveness.

        Guarantee some event triggers wake up so that we can catch Ctrl-C in case where no active messages are available.
        """
        self._heartbeat_pub.publish(Int32(data=self.get_clock().now().seconds_nanoseconds()[0]))

    def shutdown_mirror(self):
        """Shut mirror down."""
        try:
            print("    Shutting down behavior mirror ...", flush=True)
            with self._sync_lock:

                self._stopping = True
                if self._sm is not None and self._running:
                    print('    Mirror is shutting down with behavior still active!', flush=True)
                    PreemptableState.preempt = True

                    stopping_cnt = 0
                    while self._running and stopping_cnt < 200:
                        if stopping_cnt % 49 == 0:
                            print('    Waiting for mirror to stop ...')
                        stopping_cnt += 1
                        self._timing_event.wait(0.005)  # Use system time for polling check, never sim_time

                    if self._running:
                        print('    Failed to stop mirror while it is already running!', flush=True)
                        return False
                    else:
                        self._stopping = False
                        self._active_id = BehaviorSync.INVALID
                        self._sm = None
                        self._current_struct = None

                print('    Stop heartbeat timer ...', flush=True)
                self.destroy_timer(self._heartbeat_timer)
                self._timing_event.wait(0.05)
                print('    Mirror is shutdown!', flush=True)
                return True
            return False  # No active behavior

        except Exception as exc:  # pylint: disable=W0703
            print(f"Exception shutting down behavior mirror {type(exc)}\n   {exc}", flush=True)
            import traceback
            print(traceback.format_exc().replace("%", "%%"), flush=True)

    def _mirror_structure_callback(self, msg):
        Logger.loginfo(f'--> Mirror - received updated structure with checksum id = {msg.behavior_id}')
        thread = threading.Thread(target=self._activate_mirror, args=[msg])
        thread.daemon = True
        thread.start()

    def _activate_mirror(self, struct_msg):

        self.get_logger().info(f' waiting for sync to activate checksum id = {struct_msg.behavior_id}')
        with self._sync_lock:
            # Logger.loginfo(f'Got sync - try to activate checksum id = {struct_msg.behavior_id}')
            self._wait_stopping()

            if self._running:
                Logger.localwarn(f'Received a new mirror structure for checksum id={struct_msg.behavior_id} '
                                 f'while mirror is already running with active id={self._active_id}; '
                                 'adding to buffer for later')
                self._struct_buffer.append(struct_msg)
                return
            elif self._active_id not in (BehaviorSync.INVALID, struct_msg.behavior_id):
                Logger.localwarn(f'Received mirror structure id={struct_msg.behavior_id} that does '
                                 f'not match active id = {self._active_id} - will ignore!')
                return

            # At this point, either active_id is invalid or same behavior checksum id
            Logger.loginfo(f'Process the updated mirror structure for checksum id = {struct_msg.behavior_id}')
            self._active_id = struct_msg.behavior_id  # in case invalid
            self._struct_buffer = []
            self._mirror_state_machine(struct_msg)
            if self._sm:
                Logger.localinfo(f'Mirror built for checksum id = {self._active_id}')
            else:
                Logger.localwarn(f'Error processing mirror structure for behavior checksum id = {struct_msg.behavior_id}')
                Logger.logwarn('Requesting a new mirror structure from onboard ...')
                self._pub.publish('flexbe/request_mirror_structure', Int32(data=struct_msg.behavior_id))
                self._active_id = struct_msg.behavior_id
                return

            self._running = True  # Ready to start execution, so flag it as so before releasing sync

        # Release sync lock and execute the mirror
        Logger.localinfo(f'--> Mirror - begin execution of '
                         f'active mirror for checksum id = {struct_msg.behavior_id}')

        try:
            self._execute_mirror()
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f'Exception in mirror_callback: {type(exc)} ...\n  {exc}')
            Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")
            self._running = False

    def _status_callback(self, msg):
        if msg.code == BEStatus.STARTED:
            self.get_logger().info(f'--> Mirror - received BEstate={msg.code} - '
                                   f'start mirror with checksum id = {msg.behavior_id}')
            thread = threading.Thread(target=self._start_mirror, args=[msg])
            thread.daemon = True
            thread.start()
        elif self._sm:
            self.get_logger().info(f'--> Mirror - received BEstate={msg.code} with active SM - stop  current mirror')
            thread = threading.Thread(target=self._stop_mirror, args=[msg])
            thread.daemon = True
            thread.start()

    def _start_mirror(self, msg):
        self.get_logger().info(f'--> Mirror - request to start mirror with checksum id = {msg.behavior_id}')
        with self._sync_lock:
            # self.get_logger().info(f'--> Mirror - starting mirror for {msg.behavior_id} with sync lock ...')
            self._wait_stopping()

            if self._running:
                Logger.localwarn(f"Tried to start mirror for id={msg.behavior_id} while "
                                 f"mirror for id={self._active_id} is already running, will ignore.")
                return

            if len(msg.args) > 0:
                self._starting_path = "/" + msg.args[0][1:].replace("/", "_mirror/") + "_mirror"

            self._active_id = msg.behavior_id

            if len(self._struct_buffer) > 0:
                Logger.localinfo(f"Building mirror structure for checksum id={msg.behavior_id} "
                                 f"current len(struct_buffer)={len(self._struct_buffer)} ...")
                while self._sm is None and len(self._struct_buffer) > 0:
                    struct = self._struct_buffer[0]
                    self._struct_buffer = self._struct_buffer[1:]
                    if struct.behavior_id == self._active_id:
                        self._mirror_state_machine(struct)
                        Logger.localinfo(f"Mirror built for checksum '{self._active_id}'")
                    else:
                        Logger.logwarn('Discarded mismatching buffered structure for checksum %d'
                                       % (struct.behavior_id))
            # else:
            #     Logger.localinfo(f"No existing structure buffer for checksum id={msg.behavior_id} "
            #                      f"request updated structure from onboard!")

            if self._sm is None:
                Logger.localwarn(f'Missing correct mirror structure for starting behavior checksum id ={msg.behavior_id}')
                Logger.logwarn('Requesting mirror structure from onboard ...')
                self._pub.publish('flexbe/request_mirror_structure', Int32(data=msg.behavior_id))
                self._active_id = msg.behavior_id
                return

            self._running = True  # Ready to execute, so flag as running before releasing sync lock

        try:
            Logger.localinfo(f"Begin mirror execution for checksum '{self._active_id}' ...")
            self._execute_mirror()
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f'Exception in start_mirror: {type(exc)} ...\n  {exc}')
            Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")

    def _stop_mirror(self, msg):
        self.get_logger().info('--> Mirror - request to stop mirror for '
                               f'checksum id={msg.behavior_id} - waiting for sync lock ...')
        with self._sync_lock:
            # self.get_logger().info(f'--> Mirror - stopping mirror for checksum id={msg.behavior_id} with sync lock ...')
            self._stopping = True
            if self._sm is not None and self._running:
                if msg is not None and msg.code == BEStatus.FINISHED:
                    Logger.loginfo('Onboard behavior finished successfully.')
                    self._pub.publish('flexbe/behavior_update', String())
                elif msg is not None and msg.code == BEStatus.SWITCHING:
                    self._starting_path = None
                    Logger.loginfo('Onboard performing behavior switch.')
                elif msg is not None and msg.code == BEStatus.READY:
                    Logger.loginfo('Onboard engine just started, stopping currently running mirror.')
                    self._pub.publish('flexbe/behavior_update', String())
                elif msg is not None:
                    Logger.logwarn('Onboard behavior failed!')
                    self._pub.publish('flexbe/behavior_update', String())

                self._wait_stop_running()

            else:
                Logger.localinfo('No onboard behavior is active.')

            self._active_id = BehaviorSync.INVALID
            self._sm = None
            self._current_struct = None
            self._sub.remove_last_msg(self._outcome_topic, clear_buffer=True)

            if msg is not None and msg.code != BEStatus.SWITCHING:
                Logger.localinfo('\033[92m--- Behavior Mirror ready! ---\033[0m')

            # self.get_logger().info('--> Mirror - stopped mirror for '
            #                        f'checksum id={msg.behavior_id} - ready to release sync lock ...')
            self._stopping = False

    def _sync_callback(self, msg):
        if msg.behavior_id == self._active_id:
            self.get_logger().info(f'--> Mirror - sync request for checksum id={msg.behavior_id} - restart mirror')
            thread = threading.Thread(target=self._restart_mirror, args=[msg])
            thread.daemon = True
            thread.start()
        else:
            Logger.error('Cannot synchronize! Different behavior is running onboard, '
                         'please stop execution while we reset the mirror!')
            self.get_logger().info(f"Cannot synchronize!  onboard checksum id={msg.behavior_id} active={self._active_id}")
            thread = threading.Thread(target=self._stop_mirror, args=[None])
            thread.daemon = True
            thread.start()

    def _heartbeat_callback(self, msg):
        """Use heartbeat to monitor for persistent sync issues."""
        if self._active_id == BehaviorSync.INVALID:
            return  # do not check sync status if no behavior is active

        if msg.behavior_id == self._active_id:
            if self._sm is not None:
                mirror_status = self._sm.get_latest_status()
                mirror_status.behavior_id = self._active_id

                if msg.current_state_checksum != mirror_status.current_state_checksum:
                    if self._sync_heartbeat_mismatch_counter > 0:
                        # Two consecutive out of sync heartbeats
                        if msg.current_state_checksum in self._state_checksums:
                            onboard_state_path = self._state_checksums[msg.current_state_checksum]
                        else:
                            onboard_state_path = "Unknown"

                    if self._sync_heartbeat_mismatch_counter % 5 == 1:
                        Logger.error(f'OCS is possibly out of sync - onboard state is {onboard_state_path}\n'
                                     f'    Check UI and consider manual re-sync!\n'
                                     '    (mismatch may be temporarily understandable for rapidly changing outcomes)'
                                     f' {self._sync_heartbeat_mismatch_counter}')
                        Logger.localinfo(f'{msg.behavior_id} {self._active_id} : {msg.current_state_checksum}'
                                         f' {mirror_status.current_state_checksum}')
                    else:
                        # Start counting mismatches
                        self._sync_heartbeat_mismatch_counter = 1
                else:
                    # Reset mismatch counter
                    self._sync_heartbeat_mismatch_counter = 0
            elif self._active_id != 0:
                Logger.warning(f'Received matching behavior id {msg.behavior_id} with no mirror state machine active!')
            else:
                Logger.localinfo(f'Received invalid behavior id {msg.behavior_id} with active id = {self._active_id} active!')

        elif msg.INVALID not in (msg.behavior_id, self._active_id):
            if self._sync_heartbeat_mismatch_counter % 10 == 1:
                Logger.error('Out of sync! Different behavior is running onboard, please stop execution! '
                             f'{self._sync_heartbeat_mismatch_counter}')
            self._sync_heartbeat_mismatch_counter += 1
        else:
            self._sync_heartbeat_mismatch_counter += 1
            if self._sync_heartbeat_mismatch_counter % 10 == 1:
                Logger.warning('Mismatched behavior ids - please restart behavior! '
                               f'{self._sync_heartbeat_mismatch_counter}')

    def _wait_stop_running(self):
        PreemptableState.preempt = True
        running_cnt = 1
        while self._running:
            if running_cnt % 50 == 0:
                Logger.logwarn('Waiting for another mirror to stop ...')
            running_cnt += 1
            self._timing_event.wait(0.02)  # Use system time for polling check, never sim_time
        Logger.loginfo('Mirror stopped running !')

    def _wait_stopping(self):
        stopping_cnt = 1
        while self._stopping:
            if stopping_cnt % 50 == 0:
                Logger.logwarn(f'Waiting for another mirror with {self._active_id} to stop ...')
            stopping_cnt += 1
            self._timing_event.wait(0.02)  # use wall clock not sim time

    def _restart_mirror(self, msg):
        Logger.localinfo('Wait for sync lock to restart mirror for synchronization of behavior {msg.behavior_id}...')
        with self._sync_lock:
            Logger.loginfo('Restarting mirror for synchronization of behavior checksum id ={msg.behavior_id}...')
            self._sub.remove_last_msg(self._outcome_topic, clear_buffer=True)
            if self._sm is not None and self._running:
                self._wait_stop_running()
                self._sm = None

            if msg.current_state_checksum in self._state_checksums:
                current_state_path = self._state_checksums[msg.current_state_checksum]
                self._starting_path = "/" + current_state_path[1:].replace("/", "_mirror/") + "_mirror"
                Logger.loginfo(f"Current state: {current_state_path}")
            try:
                self._mirror_state_machine(self._current_struct)
                if self._sm:
                    Logger.loginfo(f'Mirror built for behavior checksum id = {msg.behavior_id}.')
                else:
                    Logger.localwarn(f'Missing correct mirror structure for restarting behavior checksum id ={msg.behavior_id}')
                    Logger.logwarn('Requesting mirror structure from onboard ...')
                    self._pub.publish('flexbe/request_mirror_structure', Int32(data=msg.behavior_id))
                    self._active_id = msg.behavior_id
                    return

            except (AttributeError, RuntimeError):
                Logger.loginfo(f'Stopping synchronization because behavior{msg.behavior_id} has stopped.')

        try:
            Logger.localinfo('Execute mirror after sync lock of restart mirror'
                             f' for synchronization of behavior {msg.behavior_id}...')
            self._execute_mirror()
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f'Exception in restart_mirror: {type(exc)} ...\n  {exc}')
            Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")

    def _execute_mirror(self):
        self._running = True

        Logger.loginfo("Executing mirror...")
        if self._starting_path is not None:
            LockableStateMachine.path_for_switch = self._starting_path
            Logger.loginfo("Starting mirror in state " + self._starting_path)
            self._starting_path = None

        result = 'preempted'
        try:
            result = self._sm.spin()
            Logger.loginfo(f"Mirror for active id = {self._active_id} finished with result '{result}'")
        except Exception as exc:
            try:
                Logger.logerr('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc))
                Logger.localerr(traceback.format_exc().replace("%", "%%"))
            except Exception:  # pylint: disable=W0703
                # Likely the loggers are dead if we ctrl-C'd during active behavior
                # so just try a simple print
                print('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc), flush=True)
                print(traceback.format_exc().replace("%", "%%"), flush=True)
            result = 'preempted'

        self._running = False

    def _mirror_state_machine(self, msg):
        try:
            self._current_struct = msg
            self._state_checksums = {}
            root = None
            for con_msg in msg.containers:
                if con_msg.path.find('/') == -1:
                    root = con_msg.path
                    break
            self._add_node(msg, root)
            # calculate checksums of all states
            for con_msg in msg.containers:
                if con_msg.path.find('/') != -1:
                    self._state_checksums[zlib.adler32(con_msg.path.encode()) & 0x7fffffff] = con_msg.path
        except Exception as exc:
            self.get_logger().warn(f"_mirror_statemachine Exception: {type(exc)} - {exc}")

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
            sm = MirrorStateMachine(outcomes=sm_outcomes)
            with sm:
                for child in container.children:
                    self._add_node(msg, path + '/' + child)
            if len(transitions) > 0:
                container_transitions = {}
                for i in range(len(container.transitions)):
                    container_transitions[sm_outcomes[i]] = transitions[container.outcomes[i]]
                MirrorStateMachine.add(container_name + '_mirror', sm, transitions=container_transitions)
            else:
                self._sm = sm
        else:
            MirrorStateMachine.add(container_name + '_mirror',
                                   MirrorState(container_name, path, container.outcomes, container.autonomy),
                                   transitions=transitions)

    def _preempt_callback(self, msg):
        # pylint: disable=unused-argument
        if self._sm is not None:
            Logger.logwarn('Explicit preempting is currently ignored, mirror should be preempted by onboard behavior.')
        else:
            Logger.logwarn('Could not preempt mirror because it seems not to be running!')
