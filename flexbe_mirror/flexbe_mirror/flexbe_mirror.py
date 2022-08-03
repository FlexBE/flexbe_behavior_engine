import threading
import zlib
import time

from rclpy.node import Node

from flexbe_core import Logger
from flexbe_core.core import PreemptableState, PreemptableStateMachine, LockableStateMachine
from .mirror_state import MirrorState

from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_msgs.msg import ContainerStructure, BehaviorSync, BEStatus
from std_msgs.msg import Empty, String, Int32, UInt8

import traceback


class FlexbeMirror(Node):

    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('flexbe_mirror')

        self._sm = None
        ProxyPublisher._initialize(self)
        ProxySubscriberCached._initialize(self)
        Logger.initialize(self)

        MirrorState.initialize_ros(self)
        PreemptableState.initialize_ros(self)
        PreemptableStateMachine.initialize_ros(self)
        LockableStateMachine.initialize_ros(self)

        # set up proxys for sm <--> GUI communication
        # publish topics
        self._pub = ProxyPublisher({'flexbe/behavior_update': String,
                                    'flexbe/request_mirror_structure': Int32})

        self._running = False
        self._stopping = False
        self._active_id = BehaviorSync.INVALID
        self._starting_path = None
        self._current_struct = None
        self._struct_buffer = list()
        self._sync_lock = threading.Lock()

        self._outcome_topic = 'flexbe/mirror/outcome'

        # listen for mirror message
        self._sub = ProxySubscriberCached()
        self._sub.subscribe(self._outcome_topic, UInt8, id=id(self))
        self._sub.enable_buffer(self._outcome_topic)

        self._sub.subscribe('flexbe/status', BEStatus, self._status_callback, id=id(self))
        self._sub.subscribe('flexbe/mirror/structure', ContainerStructure, self._mirror_callback, id=id(self))
        self._sub.subscribe('flexbe/mirror/sync', BehaviorSync, self._sync_callback, id=id(self))
        self._sub.subscribe('flexbe/mirror/preempt', Empty, self._preempt_callback, id=id(self))
        self._sub.subscribe('flexbe/heartbeat', BehaviorSync, self._heartbeat_callback, id=id(self))
        self._sync_heartbeat_mismatch_counter = 0
        Logger.loginfo('--> Mirror - ready!')

    def _mirror_callback(self, msg):
        Logger.loginfo('--> Mirror - received updated structure')

        rate = self.create_rate(10, self.get_clock())
        while self._stopping:
            rate.sleep()

        if self._running:
            Logger.logwarn('Received a new mirror structure while mirror is already running, '
                           'adding to buffer (checksum: %s).' % str(msg.behavior_id))
        elif self._active_id != BehaviorSync.INVALID and msg.behavior_id != self._active_id:
            Logger.logwarn('Checksum of received mirror structure (%s) does not match expected (%s), '
                           'will ignore.' % (str(msg.behavior_id), str(self._active_id)))
            return
        else:
            Logger.loginfo('Received a new mirror structure for checksum %s' % str(msg.behavior_id))

        self._struct_buffer.append(msg)

        if self._active_id == msg.behavior_id:
            self._struct_buffer = list()
            self._mirror_state_machine(msg)
            Logger.loginfo('Mirror built.')

            Logger.loginfo('--> Mirror - begin execution of already active mirror for checksum %s' % str(
                msg.behavior_id))

            self._execute_mirror()

    def _status_callback(self, msg):
        if msg.code == BEStatus.STARTED:
            thread = threading.Thread(target=self._start_mirror, args=[msg])
            thread.daemon = True
            thread.start()
        elif self._sm:
            self.get_logger().info(f'--> Mirror - received BEstate={msg.code} with active SM - stop mirror')
            thread = threading.Thread(target=self._stop_mirror, args=[msg])
            thread.daemon = True
            thread.start()

    def _start_mirror(self, msg):
        self.get_logger().info('--> Mirror - request to start mirror')
        with self._sync_lock:
            rate = self.create_rate(10, self.get_clock())
            while self._stopping:
                rate.sleep()

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
                    Logger.logwarn('Discarded mismatching buffered structure for checksum %s' % str(
                        struct.behavior_id))

            if self._sm is None:
                Logger.logwarn('Missing correct mirror structure, requesting...')
                time.sleep(0.2)
                # no clean way to wait for publisher to be ready...
                self._pub.publish('flexbe/request_mirror_structure', Int32(msg.behavior_id))
                self._active_id = msg.behavior_id
                return

        self._execute_mirror()

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
                rate = self.create_rate(10, self.get_clock())
                while self._running:
                    rate.sleep()
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
        """
        Using heartbeat to monitor for persistent sync issues
        """
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
            elif self._active_id != 0 :
                Logger.warning(f'Received matching behavior id {msg.behavior_id} with no state machine mirror active!')
            else:
                Logger.localinfo(f'Received matching behavior id {msg.behavior_id} with no state machine mirror active!')

        elif msg.behavior_id != msg.INVALID and self._active_id != msg.INVALID:
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
            Logger.loginfo('Restarting mirror for synchronization...')
            self._sub.remove_last_msg(self._outcome_topic, clear_buffer=True)
            if self._sm is not None and self._running:
                PreemptableState.preempt = True
                rate = self.create_rate(100, self.get_clock())
                while self._running:
                    rate.sleep()
                self._sm = None
            if msg.current_state_checksum in self._state_checksums:
                current_state_path = self._state_checksums[msg.current_state_checksum]
                self._starting_path = "/" + current_state_path[1:].replace("/", "_mirror/") + "_mirror"
                Logger.loginfo('Current state: %s' % current_state_path)
            try:
                self._mirror_state_machine(self._current_struct)
                Logger.loginfo('Mirror built.')
            except (AttributeError, RuntimeError):
                Logger.loginfo('Stopping synchronization because behavior has stopped.')

        self._execute_mirror()

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
        except Exception as e:
            Logger.logerr('(Traceback): Caught exception on preempt:\n%s' % str(e))
            Logger.logerr(traceback.format_exc())
            result = 'preempted'

        self._running = False

        Logger.loginfo('Mirror finished with result ' + result)

    def _mirror_state_machine(self, msg):
        self._current_struct = msg
        self._state_checksums = dict()
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
        container_name = path_frags[len(path_frags)-1]
        if len(container.children) > 0:
            sm_outcomes = []
            for outcome in container.outcomes:
                sm_outcomes.append(outcome + '_mirror')
            sm = PreemptableStateMachine(outcomes=sm_outcomes)
            with sm:
                for child in container.children:
                    self._add_node(msg, path+'/'+child)
            if len(transitions) > 0:
                container_transitions = {}
                for i in range(len(container.transitions)):
                    container_transitions[sm_outcomes[i]] = transitions[container.outcomes[i]]
                PreemptableStateMachine.add(container_name + '_mirror', sm, transitions=container_transitions)
            else:
                self._sm = sm
        else:
            PreemptableStateMachine.add(container_name + '_mirror',
                                        MirrorState(container_name, path, container.outcomes, container.autonomy),
                                        transitions=transitions)

    def _preempt_callback(self, msg):
        if self._sm is not None:
            Logger.logwarn('Explicit preempting is currently ignored, mirror should be preempted by onboard behavior.')
        else:
            Logger.logwarn('Could not preempt mirror because it seems not to be running!')
