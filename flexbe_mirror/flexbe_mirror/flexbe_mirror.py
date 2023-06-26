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
        self._sub.subscribe('flexbe/mirror/structure', ContainerStructure, self._mirror_callback, inst_id=id(self))
        self._sub.subscribe('flexbe/mirror/sync', BehaviorSync, self._sync_callback, inst_id=id(self))
        self._sub.subscribe('flexbe/mirror/preempt', Empty, self._preempt_callback, inst_id=id(self))
        self._sub.subscribe('flexbe/heartbeat', BehaviorSync, self._heartbeat_callback, inst_id=id(self))
        self._sync_heartbeat_mismatch_counter = 0

        # no clean way to wait for publisher to be ready...
        Logger.loginfo('--> Mirror - setting up publishers and subscribers ...')
        self._timing_event.wait(1.0)  # Give publishers time to initialize

        Logger.loginfo('--> Mirror - ready!')

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

                print('    Mirror is shutdown!', flush=True)
                return True
            return False  # No active behavior

        except Exception as exc:  # pylint: disable=W0703
            print(f"Exception shutting down behavior mirror {type(exc)}\n   {exc}", flush=True)
            import traceback
            print(traceback.format_exc().replace("%", "%%"), flush=True)

    def _mirror_callback(self, msg):
        Logger.loginfo('--> Mirror - received updated structure')

        stopping_cnt = 0
        while self._stopping:
            if stopping_cnt % 49 == 0:
                Logger.logwarn('Waiting for another mirror to stop ...')
            stopping_cnt += 1
            self._event.wait(0.02)  # use wall clock not sim time

        if self._running:
            Logger.logwarn('Received a new mirror structure while mirror is already running, '
                           'adding to buffer (checksum: %s).' % str(msg.behavior_id))
        elif self._active_id not in (BehaviorSync.INVALID, msg.behavior_id):
            Logger.logwarn('Checksum of received mirror structure (%s) does not match expected (%s), '
                           'will ignore.' % (str(msg.behavior_id), str(self._active_id)))
            return
        else:
            Logger.loginfo('Received a new mirror structure for checksum %s' % str(msg.behavior_id))

        self._struct_buffer.append(msg)

        if self._active_id == msg.behavior_id:
            self._struct_buffer = []
            self._mirror_state_machine(msg)
            Logger.loginfo('Mirror built.')

            Logger.loginfo('--> Mirror - begin execution of already active mirror for checksum %s' % str(
                msg.behavior_id))

            try:
                self._execute_mirror()
            except Exception as exc:  # pylint: disable=W0703
                Logger.logerr(f'Exception in mirror_callback: {type(exc)} ...\n  {exc}')
                Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")

    def _status_callback(self, msg):
        if msg.code == BEStatus.STARTED:
            thread = threading.Thread(target=self._start_mirror, args=[msg])
            thread.daemon = True
            thread.start()
        elif self._sm:
            self.get_logger().info(f'--> Mirror - received BEstate={msg.code} with active SM - stop  current mirror')
            thread = threading.Thread(target=self._stop_mirror, args=[msg])
            thread.daemon = True
            thread.start()

    def _start_mirror(self, msg):
        self.get_logger().info('--> Mirror - request to start mirror')
        with self._sync_lock:
            stopping_cnt = 0
            while self._stopping:
                if stopping_cnt % 49 == 0:
                    Logger.logwarn('Waiting for another mirror to stop before starting {msg.behavior_id}...')
                stopping_cnt += 1
                self._timing_event.wait(0.02)  # Use system time for polling check, never sim_time

            if self._running:
                Logger.logwarn('Tried to start mirror while it is already running, will ignore.')
                return

            if len(msg.args) > 0:
                self._starting_path = "/" + msg.args[0][1:].replace("/", "_mirror/") + "_mirror"

            self._active_id = msg.behavior_id

            while self._sm is None and len(self._struct_buffer) > 0:
                struct = self._struct_buffer[0]
                self._struct_buffer = self._struct_buffer[1:]
                if struct.behavior_id == self._active_id:
                    self._mirror_state_machine(struct)
                    Logger.loginfo('Mirror built for checksum %s.' % self._active_id)
                else:
                    Logger.logwarn('Discarded mismatching buffered structure for checksum %s'
                                   % str(struct.behavior_id))

            if self._sm is None:
                Logger.logwarn('Missing correct mirror structure, requesting...')
                self._pub.publish('flexbe/request_mirror_structure', Int32(data=msg.behavior_id))
                self._active_id = msg.behavior_id
                return

        try:
            self._execute_mirror()
        except Exception as exc:  # pylint: disable=W0703
            Logger.logerr(f'Exception in start_mirror: {type(exc)} ...\n  {exc}')
            Logger.localerr(f"{traceback.format_exc().replace('%', '%%')}")

    def _stop_mirror(self, msg):
        self.get_logger().info('--> Mirror - request to stop mirror')
        with self._sync_lock:
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

                PreemptableState.preempt = True
                running_cnt = 0
                while self._running:
                    if running_cnt % 100 == 49:
                        Logger.logwarn('Waiting for another mirror to stop running before starting this one ...')
                    running_cnt += 1
                    self._timing_event.wait(0.02)  # Use system time for polling check, never sim_time
                Logger.loginfo('Mirror stopped running !')

            else:
                Logger.loginfo('No onboard behavior is active.')

            self._active_id = BehaviorSync.INVALID
            self._sm = None
            self._current_struct = None
            self._sub.remove_last_msg(self._outcome_topic, clear_buffer=True)

            if msg is not None and msg.code != BEStatus.SWITCHING:
                Logger.loginfo('\033[92m--- Behavior Mirror ready! ---\033[0m')

            self._stopping = False

    def _sync_callback(self, msg):
        if msg.behavior_id == self._active_id:
            self.get_logger().info('--> Mirror - sync request for %s' % msg.behavior_id)
            thread = threading.Thread(target=self._restart_mirror, args=[msg])
            thread.daemon = True
            thread.start()
        else:
            Logger.error('Cannot synchronize! Different behavior is running onboard, please stop execution!')
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
                Logger.warning(f'Received matching behavior id {msg.behavior_id} with no state machine mirror active!')
            else:
                Logger.localinfo(f'Received matching behavior id {msg.behavior_id} with no state machine mirror active!')

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

    def _restart_mirror(self, msg):
        with self._sync_lock:
            Logger.loginfo('Restarting mirror for synchronization of behavior {msg.behavior_id}...')
            self._sub.remove_last_msg(self._outcome_topic, clear_buffer=True)
            if self._sm is not None and self._running:
                PreemptableState.preempt = True
                running_cnt = 0
                while self._running:
                    if running_cnt % 49 == 0:
                        Logger.logwarn('Waiting for another mirror to stop ...')
                    running_cnt += 1
                    self._timing_event.wait(0.02)  # Use system time for polling check, never sim_time
                self._sm = None

            if msg.current_state_checksum in self._state_checksums:
                current_state_path = self._state_checksums[msg.current_state_checksum]
                self._starting_path = "/" + current_state_path[1:].replace("/", "_mirror/") + "_mirror"
                Logger.loginfo(f"Current state: {current_state_path}")
            try:
                self._mirror_state_machine(self._current_struct)
                Logger.loginfo('Mirror built.')
            except (AttributeError, RuntimeError):
                Logger.loginfo('Stopping synchronization because behavior{msg.behavior_id} has stopped.')

        try:
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
            Logger.loginfo(f"Mirror finished with result '{result}'")
        except Exception as exc:
            try:
                Logger.logerr('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc))
                Logger.localerr(traceback.format_exc().replace("%", "%%"))
            except Exception:  # pylint: disable=W0703
                # Likely the loggers are dead if we ctrl-C'd during active behavior
                # so just try a simple print
                print('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc))
                print(traceback.format_exc().replace("%", "%%"))
            result = 'preempted'

        self._running = False

    def _mirror_state_machine(self, msg):
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
