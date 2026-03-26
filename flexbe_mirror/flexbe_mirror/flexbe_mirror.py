# Copyright 2024 Philipp Schillinger, Team ViGIR, Christopher Newport University
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
import time
import traceback
from collections import deque

try:
    from prctl import set_name as set_thread_name
except ImportError:
    def set_thread_name(name):
        """Set thread name dummy function if prctl not defined."""
        # print('Python thread names are not visible in ps/top unless you install prctl')
        pass

from flexbe_core import Logger, MIN_UI_VERSION, initialize_flexbe_core
from flexbe_core.core import LockableStateMachine, OperatableStateMachine
from flexbe_core.core import PreemptableState, State, StateMap
from flexbe_core.core import SyncError, TransitionError
from flexbe_core.core import map_exception_to_bestatus
from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_core.proxy.qos import QOS_OUTCOME

from flexbe_msgs.msg import BEStatus, BehaviorSync, ContainerStructure, StateMapMsg

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import Empty, Int32, String, UInt32

from .mirror_concurrency_container import MirrorConcurrencyContainer
from .mirror_priority_container import MirrorPriorityContainer
from .mirror_state import MirrorState
from .mirror_state_machine import MirrorStateMachine

bestatus_map = {
    BEStatus.STARTED: 'STARTED',
    BEStatus.FINISHED: 'FINISHED',
    BEStatus.FAILED: 'FAILED',
    BEStatus.LOCKED: 'LOCKED',
    BEStatus.WAITING: 'WAITING',
    BEStatus.SWITCHING: 'SWITCHING',
    BEStatus.WARNING: 'WARNING',
    BEStatus.ERROR: 'ERROR',
    BEStatus.READY: 'READY',
    BEStatus.RUNNING: 'RUNNING',
    BEStatus.STOPPED: 'STOPPED',
}


class FlexbeMirror(Node):
    """Class to handle the FlexBE mirror of onboard statemachine."""

    def __init__(self):
        """Initiate the Node class's constructor and give it a name."""
        super().__init__('flexbe_mirror')

        self._sm = None
        initialize_flexbe_core(self)

        self._timing_event = threading.Event()  # Used for wait timer

        # Keep track of mirror thread status
        # starting one while other is stopping is valid,
        # but only one thread should be running at a time
        self._starting = False
        self._running = False
        self._stopping = False
        self._shutdown_requested = False

        self._last_obe_status = None
        self._start_requested = False
        self._last_stop_behavior_id = BehaviorSync.INVALID
        self._last_stop_status_code = None
        self._active_id = BehaviorSync.INVALID
        self._starting_path = None
        self._pending_start_behavior_id = BehaviorSync.INVALID
        self._pending_start_args = []
        self._current_struct = None
        self._struct_buffer = deque()
        self._pending_terminal_status_code = None
        self._pending_terminal_status_behavior_id = BehaviorSync.INVALID
        self._pending_terminal_status_args = []
        self._sync_lock = threading.Lock()
        self._state_map = None
        self._last_onboard_mismatch_sig = None
        self._last_mirror_mismatch_sig = None
        self._system_clock = Clock()
        self._active_thread_start = None
        self._wait_timeout_sec = float(self.declare_parameter('mirror_wait_timeout_sec', 1.0).value)
        self._wait_poll_sec = float(self.declare_parameter('mirror_wait_poll_sec', 0.001).value)
        self._soft_stop_timeout_sec = float(self.declare_parameter('mirror_soft_stop_timeout_sec', 0.0).value)
        self._soft_stop_stall_sec = float(self.declare_parameter('mirror_soft_stop_stall_sec', 0.1).value)
        self._soft_stop_poll_sec = float(self.declare_parameter('mirror_soft_stop_poll_sec', 0.01).value)
        self._soft_stop_requested = False
        self._soft_stop_thread = None

        # set up proxies for sm <--> GUI communication
        # publish topics
        self._heartbeat_pub = self.create_publisher(Int32, Topics._MIRROR_HEARTBEAT_TOPIC, 2)
        # Keep enough transient-local history for reconnecting consumers to observe
        # the complete terminal/startup lifecycle around restarts and switches.
        latching_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        status_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._mirror_status_pub = self.create_publisher(BEStatus, Topics._MIRROR_STATUS_TOPIC, qos_profile=latching_qos)
        self._mirror_sync_warning_active = False
        self._version_sub = self.create_subscription(String, Topics._UI_VERSION_TOPIC,
                                                     self._version_callback, qos_profile=latching_qos)
        self._request_struct_pub = self.create_publisher(Int32, Topics._REQUEST_STRUCTURE_TOPIC, 2)

        # listen for mirror control messages using standard subscriptions
        self._status_sub = self.create_subscription(BEStatus, Topics._ONBOARD_STATUS_TOPIC,
                                                    self._status_callback, qos_profile=status_qos)
        self._struct_sub = self.create_subscription(ContainerStructure, Topics._MIRROR_STRUCTURE_TOPIC,
                                                    self._mirror_structure_callback, 10)
        self._sync_sub = self.create_subscription(BehaviorSync, Topics._MIRROR_SYNC_TOPIC, self._sync_callback, 10)
        self._preempt_sub = self.create_subscription(Empty, Topics._MIRROR_PREEMPT_TOPIC, self._preempt_callback, 10)
        self._onboard_heartbeat_sub = self.create_subscription(BehaviorSync, Topics._ONBOARD_HEARTBEAT_TOPIC,
                                                               self._onboard_heartbeat_callback, 10)
        self._sync_heartbeat_mismatch_counter = 0

        # Use proxy publisher/subscriber for access in states
        # but just initialize here once for all
        self._beh_update_pub = ProxyPublisher({Topics._BEHAVIOR_UPDATE_TOPIC: Int32})

        self._outcome_sub = ProxySubscriberCached()
        self._outcome_sub.subscribe(Topics._OUTCOME_TOPIC, UInt32, qos=QOS_OUTCOME, inst_id=id(self))
        self._outcome_sub.enable_buffer(Topics._OUTCOME_TOPIC)

        self._state_map_pub = self.create_publisher(StateMapMsg, Topics._STATE_MAP_OCS_TOPIC, latching_qos)

        # no clean way to wait for publisher to be ready...
        Logger.loginfo('--> Mirror - setting up publishers and subscribers ...')
        threading.Event().wait(0.25)  # Give publishers time to initialize

        # Require periodic events in case behavior is not connected to allow orderly shutdown
        self._heartbeat_timer = self.create_timer(2.0, self.heartbeat_timer_callback)

        Logger.loginfo('--> Mirror - ready!')
        self._publish_mirror_status(BEStatus.READY)

    def _version_callback(self, msg):
        """Check version required by behavior launcher."""
        vui = FlexbeMirror._parse_version(msg.data)
        vex = FlexbeMirror._parse_version(MIN_UI_VERSION)
        if vui < vex:
            Logger.logwarn('FlexBE WebUI needs to be updated!\n'
                           f'Mirror requires at least version {MIN_UI_VERSION}, '
                           f' but you have {msg.data}\n'
                           'Please update the flexbe_webui software.')

    @staticmethod
    def _parse_version(v):
        """Extract version information."""
        result = 0
        offset = 1
        for n in reversed(v.split('.')):
            result += int(n) * offset
            offset *= 1000
        return result

    def get_elapsed_str(self, start_time):
        """Return a truncated time string for debugging."""
        elapsed = self._system_clock.now() - start_time
        sec, nsec = start_time.seconds_nanoseconds()
        return f'started at {sec & 0xFFFF}.{nsec // 1000:06d} s (elapsed={elapsed.nanoseconds / 1e9} s)'

    def _notify_state_change(self):
        """Wake waiters blocked on mirror lifecycle state changes."""
        self._timing_event.set()

    def _publish_mirror_status(self, code, behavior_id=None, args=None):
        """Publish mirror lifecycle or sync status on the dedicated mirror status topic."""
        status_msg = BEStatus(code=code)
        if behavior_id is not None:
            status_msg.behavior_id = behavior_id
        if args is not None:
            status_msg.args = [str(arg) for arg in args]

        try:
            status_msg.stamp = self.get_clock().now().to_msg()
        except Exception:  # pylint: disable=W0703
            pass

        self._mirror_status_pub.publish(status_msg)

    def _set_sync_warning_state(self, active, behavior_id=None):
        """Publish WARNING once per confirmed sync issue, then RUNNING once when recovered."""
        if active:
            if self._mirror_sync_warning_active:
                return
            self._mirror_sync_warning_active = True
            self._publish_mirror_status(BEStatus.WARNING, behavior_id=behavior_id)
            return

        if not self._mirror_sync_warning_active:
            return

        self._mirror_sync_warning_active = False
        if behavior_id not in (None, BehaviorSync.INVALID) and self._running and not self._stopping:
            self._publish_mirror_status(BEStatus.RUNNING, behavior_id=behavior_id)

    def _clear_sync_warning_state(self):
        """Clear the sync warning latch without publishing a recovery status."""
        self._mirror_sync_warning_active = False

    def _record_terminal_status(self, msg):
        """Remember the last terminal onboard status until STOPPED completes the run."""
        self._pending_terminal_status_code = msg.code
        self._pending_terminal_status_behavior_id = msg.behavior_id
        self._pending_terminal_status_args = list(msg.args)

    def _clear_terminal_status(self):
        """Forget any pending terminal status for the active run."""
        self._pending_terminal_status_code = None
        self._pending_terminal_status_behavior_id = BehaviorSync.INVALID
        self._pending_terminal_status_args = []

    def _clear_outcome_tracking(self):
        """Drop cached outcome messages when forcing a fresh mirror start."""
        self._outcome_sub.remove_last_msg(Topics._OUTCOME_TOPIC, clear_buffer=True)

    def _mirror_transition_callback(self, _active_states):
        """Publish RUNNING whenever the mirrored active path changes."""
        if self._mirror_sync_warning_active:
            return
        behavior_id = self._active_id
        if behavior_id in (None, BehaviorSync.INVALID) or not self._running:
            return
        self._publish_mirror_status(BEStatus.RUNNING, behavior_id=behavior_id)

    def _log_compact_sync_summary(self, onboard_sig, mirror_sig):
        """Log a compact heartbeat mismatch summary without per-state detail dumps."""
        Logger.localwarn(
            'OCS heartbeat mismatch %s: onboard=%s mirror=%s',
            self._sync_heartbeat_mismatch_counter,
            onboard_sig,
            mirror_sig
        )

    def _log_exception(self, context, exc, start_time=None, default_status=BEStatus.ERROR):
        """Log exceptions consistently using mapped BEStatus severity categories."""
        status_code = map_exception_to_bestatus(exc, default=default_status)
        time_msg = '' if start_time is None else f' {self.get_elapsed_str(start_time)}'
        msg = f'{context}{time_msg}: {type(exc).__name__} - {exc}'
        if status_code in (BEStatus.WARNING, ):
            Logger.logwarn(msg)
        else:
            Logger.logerr(msg)
        trace = traceback.format_exc()
        if trace.strip() != 'NoneType: None':
            Logger.localinfo(f"{trace.replace('%', '%%')}")

    def _handle_execution_exception(self, context, exc, start_time):
        """Handle execute-path exceptions consistently and reset running flag."""
        self._log_exception(context, exc, start_time=start_time)
        with self._sync_lock:
            self._running = False
        self._clear_sync_warning_state()
        self._notify_state_change()

    def heartbeat_timer_callback(self):
        """
        Allow monitoring of Mirror liveness.

        Use negative time in seconds if mirror is not active, and loop count when running

        Guarantee some event triggers wake up so that we can catch Ctrl-C in case where no active messages are available.
        """
        Logger.check_local_enabled()  # Periodically update our local logger permissions
        heartbeat = Int32(data=-(self.get_clock().now().seconds_nanoseconds()[0] & 0x0000FFFF))
        if self._sm is not None and self._running:
            heartbeat.data = self._sm._total_loop_count

        self._heartbeat_pub.publish(heartbeat)

    def shutdown_mirror(self):
        """Shut mirror down."""
        try:
            print(f"    Shutting down behavior mirror '{self._active_id}' ...", flush=True)
            with self._sync_lock:
                self._shutdown_requested = True
                self._stopping = True
                self._notify_state_change()
                if self._sm is not None:
                    if self._running:
                        print(f"    Mirror '{self._active_id}' is shutting down with behavior still active!", flush=True)
                        self._wait_stop_running(self._system_clock.now())

                        if self._running:
                            # Re-check running after waiting to stop
                            print(f"    Failed to stop mirror '{self._active_id}' while it is already running!", flush=True)
                            return False

                self._stopping = False
                self._notify_state_change()
                self._active_id = BehaviorSync.INVALID
                self._sm = None
                self._current_struct = None

                print('    Stop heartbeat timer ...', flush=True)
                self.destroy_timer(self._heartbeat_timer)
                threading.Event().wait(0.05)

                print('   Remove proxy subscribers ...', flush=True)
                self._outcome_sub.unsubscribe_topic(Topics._OUTCOME_TOPIC, inst_id=id(self))

                print('   Remove proxy publishers ...', flush=True)
                self._beh_update_pub.remove_publisher(Topics._BEHAVIOR_UPDATE_TOPIC)

                print('    Mirror is shutdown!', flush=True)
                return True

        except Exception as exc:  # pylint: disable=W0703
            print(f"Exception shutting down behavior mirror '{type(exc)}'\n   {exc}", flush=True)
            import traceback
            print(traceback.format_exc().replace('%', '%%'), flush=True)
            return False

    def _mirror_structure_callback(self, msg):
        """Process structure message and activate the mirror."""
        if self._shutdown_requested:
            return
        start_time = self._system_clock.now()
        Logger.localinfo(f'--> Mirror - received updated structure with checksum id = {msg.behavior_id}'
                         f'   at {start_time.nanoseconds} ns')
        thread = threading.Thread(target=self._activate_mirror, args=[msg, start_time],
                                  name=f'activate_mirror_{msg.behavior_id}_{start_time.nanoseconds}')
        thread.daemon = True
        thread.start()

    def _activate_mirror(self, struct_msg, start_time):
        """Process the mirror structure and begin execution if valid."""
        set_thread_name('act' + f'{start_time.nanoseconds}'[-12:])  # only 15 chars allowed
        try:
            with self._sync_lock:
                if self._shutdown_requested:
                    return
                self._wait_stopping(start_time)
                if self._shutdown_requested:
                    return

                if self._running:
                    Logger.localwarn(f'Received a new mirror structure for checksum id={struct_msg.behavior_id} '
                                     f'while mirror is already running with active id={self._active_id}; '
                                     'adding to buffer for later!')
                    self._struct_buffer.append(struct_msg)
                    self._starting = False
                    return
                elif self._active_id not in (BehaviorSync.INVALID, struct_msg.behavior_id):
                    Logger.localwarn(f'Received mirror structure id={struct_msg.behavior_id} that does '
                                     f'not match active id = {self._active_id} - will ignore!')
                    Logger.logwarn('Ignoring SM structure with mismatched id!')
                    self._starting = False
                    return

                # At this point, either active_id is invalid or same behavior checksum id, so make it active
                self._struct_buffer.clear()  # Clear any stored structures and use this one
                self._mirror_state_machine(struct_msg)
                if self._sm:
                    Logger.localinfo(f'Mirror built for behavior id = {self._active_id}.')
                elif not self._starting:
                    Logger.logwarn(f'Error: Requesting a new mirror structure from onboard ({struct_msg.behavior_id})...')
                    self._request_struct_pub.publish(Int32(data=struct_msg.behavior_id))
                    self._starting = True  # Wait for message to retrigger
                    return
                else:
                    Logger.localinfo('Error: cannot build mirror but we have already re-requested '
                                     'structure before; ignore request to activate.')
                    return

                if not self._start_requested:
                    # Keep prepared structure cached but defer execution until BEStatus.STARTED arrives.
                    return

                # STARTED was already received and we were waiting on structure availability.
                self._active_id = struct_msg.behavior_id
                self._clear_terminal_status()
                self._mirror_sync_warning_active = False
                self._running = True
                self._starting = False
                self._publish_mirror_status(BEStatus.STARTED,
                                            behavior_id=struct_msg.behavior_id,
                                            args=self._pending_start_args)
        except SyncError as exc:
            self._log_exception('Activation wait for mirror stop failed', exc, start_time=start_time)
            return

        # Release sync lock and execute the mirror
        try:
            self._execute_mirror(start_time)
        except (SyncError, TransitionError) as exc:
            self._handle_execution_exception('Exception in activate mirror', exc, start_time)
        except Exception as exc:  # pylint: disable=W0703
            # TODO: High-risk guardrail. Revisit narrowing after proving no lifecycle regressions during restart/stop races.
            self._handle_execution_exception('Exception in activate mirror', exc, start_time)

        Logger.localwarn(f'Done executing mirror {self._active_id} from activation '
                         f'{self.get_elapsed_str(start_time)}')

    def _status_callback(self, msg):
        """Set mirror to mimic behavior engine status."""
        try:
            if self._shutdown_requested:
                return
            if self._should_ignore_duplicate_ready_stop(msg):
                return
            self._last_obe_status = msg.code
            start_time = self._system_clock.now()
            if msg.code == BEStatus.STARTED:
                # Start mirror using latest structure.
                Logger.localinfo('Mirror - received BEStatus=%s (%s) start mirror with behavior id = %s '
                                 'started at %s ns(%s, %s, %s)',
                                 bestatus_map.get(msg.code, 'UNKNOWN'), msg.code, msg.behavior_id,
                                 start_time.nanoseconds, self._starting, self._running, self._stopping)
                self._start_requested = True
                self._pending_start_behavior_id = msg.behavior_id
                self._pending_start_args = list(msg.args)
                self._last_stop_behavior_id = BehaviorSync.INVALID
                self._last_stop_status_code = None
                if self._soft_stop_requested or self._stopping:
                    Logger.localinfo('Mirror - deferring BEStatus=STARTED (%s) until current stop completes '
                                     '(%s, %s, %s)',
                                     msg.behavior_id, self._starting, self._running, self._stopping)
                    self._starting = True
                    self._notify_state_change()
                    return
                self._starting = False  # Clear starting flag on new START request
                self._soft_stop_requested = False
                thread = threading.Thread(target=self._start_mirror, args=[msg, start_time],
                                          name=f'start_mirror_{msg.behavior_id}_{start_time.nanoseconds}')
                thread.daemon = True
                thread.start()
            elif self._sm or self._active_id != BehaviorSync.INVALID:
                # We have valid state machine structure (or SM already completed but active_id not yet cleared),
                # respond according to status
                active_behavior_id = getattr(self, '_active_id', BehaviorSync.INVALID)
                if active_behavior_id == BehaviorSync.INVALID and getattr(self._sm, 'id', None) is not None:
                    active_behavior_id = self._sm.id
                if msg.code in (BEStatus.FINISHED, BEStatus.FAILED, BEStatus.ERROR):
                    if msg.behavior_id != active_behavior_id:
                        Logger.localinfo('Mirror - ignoring terminal BEStatus=%s (%s) for inactive behavior id=%s'
                                         ' while active id=%s (%s, %s, %s)',
                                         bestatus_map.get(msg.code, 'UNKNOWN'), msg.code, msg.behavior_id,
                                         active_behavior_id, self._starting, self._running, self._stopping)
                        return
                    self._record_terminal_status(msg)
                    if msg.code in (BEStatus.ERROR, BEStatus.FAILED):
                        Logger.logerr('Mirror - received terminal BEStatus=%s (%s) for active behavior id=%s; '
                                      'continue draining until STOPPED (%s, %s, %s)',
                                      bestatus_map.get(msg.code, 'UNKNOWN'), msg.code, msg.behavior_id,
                                      self._starting, self._running, self._stopping)
                    else:
                        Logger.localinfo('Mirror - received terminal BEStatus=%s (%s) for active behavior id=%s; '
                                         'continue draining until STOPPED (%s, %s, %s)',
                                         bestatus_map.get(msg.code, 'UNKNOWN'), msg.code, msg.behavior_id,
                                         self._starting, self._running, self._stopping)
                elif msg.code == BEStatus.WARNING:
                    Logger.logwarn('Mirror - ignoring non-terminal BEStatus=%s (%s) for active behavior id=%s '
                                   '(%s, %s, %s)',
                                   bestatus_map.get(msg.code, 'UNKNOWN'), msg.code, msg.behavior_id,
                                   self._starting, self._running, self._stopping)
                elif msg.code in (BEStatus.STOPPED, BEStatus.READY):
                    if msg.code == BEStatus.STOPPED and msg.behavior_id != active_behavior_id:
                        Logger.localinfo('Mirror - ignoring STOPPED for inactive behavior id=%s while active id=%s '
                                         '(%s, %s, %s)',
                                         msg.behavior_id, active_behavior_id,
                                         self._starting, self._running, self._stopping)
                        return
                    if msg.code == BEStatus.READY and msg.behavior_id not in (BehaviorSync.INVALID, active_behavior_id):
                        Logger.localinfo('Mirror - ignoring READY for inactive behavior id=%s while active id=%s '
                                         '(%s, %s, %s)',
                                         msg.behavior_id, active_behavior_id,
                                         self._starting, self._running, self._stopping)
                        return
                    # Soft-stop path: keep draining mirror outcomes while progress is observed.
                    if self._has_deferred_start_request():
                        Logger.localinfo('Mirror - preserving deferred STARTED for id=%s while processing '
                                         'BEStatus=%s (%s).',
                                         self._pending_start_behavior_id,
                                         bestatus_map.get(msg.code, 'UNKNOWN'),
                                         msg.behavior_id)
                    else:
                        self._start_requested = False
                        self._starting = False
                    self._soft_stop_requested = True
                    if not self._stopping and (self._soft_stop_thread is None or not self._soft_stop_thread.is_alive()):
                        Logger.localinfo('Mirror - received BEStatus=%s (%s) - request graceful stop (%s, %s, %s)',
                                         bestatus_map.get(msg.code, 'UNKNOWN'), msg.code,
                                         self._starting, self._running, self._stopping)
                        thread = threading.Thread(target=self._soft_stop_watchdog, args=[msg, start_time],
                                                  name=f'soft_stop_{msg.behavior_id}_{start_time.nanoseconds}')
                        thread.daemon = True
                        self._soft_stop_thread = thread
                        thread.start()
                    else:
                        Logger.localinfo('Mirror - received BEStatus=%s (%s) - graceful stop already in progress (%s, %s, %s)',
                                         bestatus_map.get(msg.code, 'UNKNOWN'), msg.code,
                                         self._starting, self._running, self._stopping)

                else:
                    # otherwise normal
                    Logger.localinfo('Mirror - received BEStatus=%s (%s) normal active mode (%s, %s, %s)',
                                     bestatus_map.get(msg.code, 'UNKNOWN'), msg.code,
                                     self._starting, self._running, self._stopping)
            else:
                # Waiting for a new behavior, should be READY message (or STARTED handled above)
                if msg.code == BEStatus.READY:
                    self._start_requested = False
                if msg.code not in (BEStatus.READY, ):
                    Logger.localinfo('Mirror - received BEStatus=%s (%s) - no active SM (%s, %s, %s)',
                                     bestatus_map.get(msg.code, 'UNKNOWN'), msg.code,
                                     self._starting, self._running, self._stopping)
                # Normal to receive periodic READY signals

        except Exception as exc:
            # TODO: High-risk guardrail. Keep broad catch until callback-thread error propagation policy is redesigned.
            self._log_exception('Exception in Mirror _status_callback callback', exc)

    def _should_ignore_duplicate_ready_stop(self, msg):
        """Ignore READY immediately following a STOPPED-driven stop for the same behavior."""
        return (
            msg.code == BEStatus.READY
            and self._last_stop_status_code == BEStatus.STOPPED
            and msg.behavior_id in (BehaviorSync.INVALID, self._last_stop_behavior_id)
        )

    def _has_deferred_start_request(self):
        """Return True when a queued STARTED is waiting for the current stop to complete."""
        return (
            self._start_requested
            and self._pending_start_behavior_id != BehaviorSync.INVALID
            and self._pending_start_behavior_id != self._active_id
        )

    @staticmethod
    def _extract_soft_stop_snapshot(state_machine):
        """Collect simple drain progress markers for graceful stop."""
        if state_machine is None:
            return None
        return (
            state_machine._total_loop_count,
            len(state_machine._pending_outcomes),
            state_machine._current_state is not None,
        )

    @staticmethod
    def _soft_stop_has_progress(previous_snapshot, current_snapshot):
        """Return True if snapshots indicate drain progress."""
        if previous_snapshot is None or current_snapshot is None:
            return True
        return current_snapshot != previous_snapshot

    @staticmethod
    def _soft_stop_is_quiescent(snapshot):
        """Return True when soft-stop snapshot shows no remaining drain work."""
        if snapshot is None:
            return False
        _loop_count, pending, current_state_active = snapshot
        return pending == 0 and not current_state_active

    def _soft_stop_watchdog(self, msg, start_time):
        """Gracefully drain mirror progress after STOPPED/READY before forcing teardown."""
        set_thread_name('sft' + f'{start_time.nanoseconds}'[-12:])  # only 15 chars allowed
        timeout_sec = max(0.0, float(self._soft_stop_timeout_sec))
        stall_sec = max(0.0, float(self._soft_stop_stall_sec))
        poll_sec = max(0.001, float(self._soft_stop_poll_sec))
        deadline = (time.monotonic() + timeout_sec) if timeout_sec > 0.0 else None
        last_progress = time.monotonic()
        previous_snapshot = None

        while True:
            with self._sync_lock:
                if not self._soft_stop_requested:
                    return
                running = self._running
                stopping = self._stopping
                state_machine = self._sm

            if stopping:
                return
            if not running:
                break

            snapshot = self._extract_soft_stop_snapshot(state_machine)
            if self._soft_stop_has_progress(previous_snapshot, snapshot):
                last_progress = time.monotonic()
            if self._soft_stop_is_quiescent(snapshot):
                Logger.localinfo('Mirror graceful-stop reached quiescent snapshot=%s for id=%s; forcing stop.',
                                 snapshot,
                                 msg.behavior_id)
                break
            previous_snapshot = snapshot

            now = time.monotonic()
            if deadline is not None and now >= deadline:
                Logger.localwarn('Mirror graceful-stop timeout for id=%s after %.3fs; forcing stop.',
                                 msg.behavior_id,
                                 timeout_sec)
                break
            if now - last_progress >= stall_sec:
                Logger.localwarn('Mirror graceful-stop stalled for id=%s with snapshot=%s; forcing stop.',
                                 msg.behavior_id,
                                 snapshot)
                break

            self._timing_event.clear()
            self._timing_event.wait(poll_sec)

        with self._sync_lock:
            if not self._soft_stop_requested:
                return

        stop_time = self._system_clock.now()
        self._stop_mirror(msg, stop_time)

    def _start_mirror(self, msg, start_time):
        """Call on STARTED command from OBE."""
        set_thread_name('str' + f'{start_time.nanoseconds}'[-12:])  # only 15 chars allowed
        try:
            with self._sync_lock:
                if self._shutdown_requested:
                    return
                self._wait_stopping(start_time)
                if self._shutdown_requested:
                    return

                if self._running:
                    previous_active_id = self._active_id
                    Logger.localwarn('Start request for id=%s received while mirror id=%s is still running; '
                                     'forcing hard stop before start.',
                                     msg.behavior_id,
                                     previous_active_id)
                    try:
                        self._wait_stop_running(start_time)
                    except SyncError as exc:
                        self._log_exception('Hard stop for STARTED failed', exc, start_time=start_time)
                        return

                    if self._sm is not None:
                        try:
                            self._sm.destroy()
                        except Exception as exc:  # pylint: disable=W0703
                            self._log_exception('Destroy mirror state-machine failure during hard STARTED stop',
                                                exc,
                                                start_time=start_time,
                                                default_status=BEStatus.WARNING)

                    self._active_id = BehaviorSync.INVALID
                    self._sm = None
                    self._current_struct = None
                    self._last_onboard_mismatch_sig = None
                    self._last_mirror_mismatch_sig = None
                    self._clear_outcome_tracking()
                    self._clear_terminal_status()

                Logger.localinfo(f' Start request mirror for {msg.behavior_id} in thread {self.get_elapsed_str(start_time)}')

                if len(msg.args) > 0:
                    self._starting_path = '/' + msg.args[0][1:].replace('/', '_mirror/') + '_mirror'

                if self._sm is not None and self._sm.id != msg.behavior_id:
                    Logger.localwarn('Discarding stale preloaded mirror id=%s before starting id=%s.',
                                     self._sm.id,
                                     msg.behavior_id)
                    self._sm = None
                    self._current_struct = None
                if len(self._struct_buffer) > 0:
                    while self._sm is None and len(self._struct_buffer) > 0:
                        # Search buffer looking for desired structure
                        struct = self._struct_buffer.popleft()
                        if struct.behavior_id == msg.behavior_id:
                            self._mirror_state_machine(struct)
                            Logger.localinfo(f"Mirror built for checksum '{msg.behavior_id}'")
                        else:
                            Logger.logwarn(f"Discarded mismatching buffered structure for checksum '{struct.behavior_id}'")

                if self._sm is None:
                    Logger.localwarn('Missing correct mirror structure for starting behavior '
                                     f"checksum id ='{msg.behavior_id}'  ({self._starting}, {self._running}, {self._stopping})")
                    if not self._starting:
                        Logger.logwarn('Requesting mirror structure from onboard ...')
                        self._request_struct_pub.publish(Int32(data=msg.behavior_id))
                    self._starting = True
                    self._notify_state_change()
                    return

                # We have a valid state machine
                self._active_id = msg.behavior_id
                self._mirror_sync_warning_active = False

                self._running = True  # Ready to execute, so flag as running before releasing sync lock
                self._starting = False
                self._publish_mirror_status(BEStatus.STARTED, behavior_id=msg.behavior_id, args=msg.args)
                self._notify_state_change()
        except SyncError as exc:
            self._log_exception('Start wait for mirror stop failed', exc, start_time=start_time)
            return

        try:
            self._execute_mirror(start_time)
        except (SyncError, TransitionError) as exc:
            self._handle_execution_exception('Exception in start_mirror', exc, start_time)
        except Exception as exc:  # pylint: disable=W0703
            # TODO: High-risk guardrail. Revisit narrowing after proving no lifecycle regressions during restart/stop races.
            self._handle_execution_exception('Exception in start_mirror', exc, start_time)

        Logger.localwarn(f"Mirror execution for '{self._active_id}' is finished "
                         f'{self.get_elapsed_str(start_time)}')

    def _stop_mirror(self, msg, start_time):
        """Stop mirror from executing current structure."""
        set_thread_name('stp' + f'{start_time.nanoseconds}'[-12:])  # only 15 chars allowed
        deferred_start_msg = None
        with self._sync_lock:
            stop_code = msg.code if msg is not None else None
            queued_start_requested = self._start_requested
            queued_start_behavior_id = self._pending_start_behavior_id
            queued_start_args = list(self._pending_start_args)
            Logger.localinfo(f"Mirror '{self._active_id}' - stopping mirror "
                             f"for checksum id={msg.behavior_id if msg is not None else 'None'} "
                             f' {self.get_elapsed_str(start_time)}')
            self._stopping = True
            self._notify_state_change()
            if self._sm is not None and self._running:
                if msg is None:
                    Logger.logwarn('Onboard behavior stop request (from sync)!')
                elif msg.code == BEStatus.STOPPED and self._pending_terminal_status_code == BEStatus.FINISHED:
                    Logger.loginfo('Onboard behavior finished successfully.')
                    self._beh_update_pub.publish(Topics._BEHAVIOR_UPDATE_TOPIC, Int32(data=-1))
                elif msg.code == BEStatus.SWITCHING:
                    self._starting_path = None
                    Logger.loginfo('Onboard performing behavior switch.')
                elif msg.code == BEStatus.READY:
                    Logger.loginfo('Onboard engine just started, stopping currently running mirror.')
                    self._beh_update_pub.publish(Topics._BEHAVIOR_UPDATE_TOPIC, Int32(data=-1))
                else:
                    Logger.logwarn('Onboard behavior failed!')
                    self._beh_update_pub.publish(Topics._BEHAVIOR_UPDATE_TOPIC, Int32(data=-1))

                try:
                    self._wait_stop_running(start_time)
                except SyncError as exc:
                    self._log_exception('Mirror stop synchronization failure', exc, start_time=start_time)
                finally:
                    if queued_start_requested and queued_start_behavior_id != BehaviorSync.INVALID:
                        self._start_requested = True
                        self._starting = True
                        self._pending_start_behavior_id = queued_start_behavior_id
                        self._pending_start_args = list(queued_start_args)

                # _execute_mirror may have completed naturally and already destroyed self._sm
                # while _sync_lock was released inside _wait_stop_running; re-check before destroy.
                if self._sm is not None:
                    try:
                        self._sm.destroy()
                    except Exception as exc:  # pylint: disable=W0703
                        # Keep mirror teardown progressing even if state-machine destroy fails.
                        self._log_exception('Mirror state-machine destroy failure during stop',
                                            exc,
                                            start_time=start_time,
                                            default_status=BEStatus.WARNING)

            elif self._sm is not None:
                # SM was built (structure arrived) but STARTED never came before the stop.
                Logger.localinfo('Stop request for mirror that was built but never started; destroying SM.')
                try:
                    self._sm.destroy()
                except Exception as exc:  # pylint: disable=W0703
                    self._log_exception('Mirror state-machine destroy failure for unstarted SM',
                                        exc, start_time=start_time, default_status=BEStatus.WARNING)
            else:
                # Mirror already completed its spin naturally before this stop arrived.
                Logger.localinfo('Stop request received after mirror already completed.')

            terminal_stop = stop_code in (BEStatus.FINISHED, BEStatus.FAILED, BEStatus.ERROR)

            if msg is not None and self._pending_terminal_status_code is not None:
                self._publish_mirror_status(self._pending_terminal_status_code,
                                            behavior_id=self._pending_terminal_status_behavior_id,
                                            args=self._pending_terminal_status_args)
            elif msg is not None and terminal_stop:
                self._publish_mirror_status(stop_code, behavior_id=msg.behavior_id, args=msg.args)
            elif msg is not None and stop_code == BEStatus.SWITCHING:
                self._publish_mirror_status(stop_code, behavior_id=msg.behavior_id, args=msg.args)

            if msg is not None and (stop_code == BEStatus.STOPPED or terminal_stop):
                self._publish_mirror_status(BEStatus.STOPPED, behavior_id=msg.behavior_id, args=msg.args)

            self._active_id = BehaviorSync.INVALID
            self._sm = None
            self._current_struct = None
            self._last_onboard_mismatch_sig = None
            self._last_mirror_mismatch_sig = None
            self._soft_stop_requested = False
            self._last_stop_behavior_id = msg.behavior_id if msg is not None else BehaviorSync.INVALID
            self._last_stop_status_code = msg.code if msg is not None else None
            self._clear_sync_warning_state()
            self._clear_terminal_status()

            if queued_start_requested and queued_start_behavior_id != BehaviorSync.INVALID and not self._shutdown_requested:
                deferred_start_msg = BEStatus(code=BEStatus.STARTED, behavior_id=queued_start_behavior_id)
                deferred_start_msg.args = list(queued_start_args)
                self._starting = True
            else:
                self._start_requested = False
                self._starting = False
                self._pending_start_behavior_id = BehaviorSync.INVALID
                self._pending_start_args = []

            if msg is not None and msg.code != BEStatus.SWITCHING:
                Logger.loginfo('\033[92m--- Behavior Mirror ready! ---\033[0m')
                self._publish_mirror_status(BEStatus.READY)
            self._stopping = False
            self._notify_state_change()

        if deferred_start_msg is not None:
            deferred_start_time = self._system_clock.now()
            thread = threading.Thread(target=self._start_mirror, args=[deferred_start_msg, deferred_start_time],
                                      name=f'start_mirror_{deferred_start_msg.behavior_id}_{deferred_start_time.nanoseconds}')
            thread.daemon = True
            thread.start()

    def _sync_callback(self, msg):
        """Call on sync request message."""
        if self._shutdown_requested:
            return
        start_time = self._system_clock.now()
        active_id = self._active_id
        sm = self._sm
        if msg.behavior_id == active_id:
            if sm is None:
                self._log_exception('Mirror synchronize request for inactive mirror',
                                    SyncError(f'id={msg.behavior_id} matched active={active_id} without active mirror'),
                                    start_time=start_time,
                                    default_status=BEStatus.WARNING)
                return
            Logger.logwarn(f'--> Mirror - sync request for behavior id={msg.behavior_id} - restart mirror')
            thread = threading.Thread(target=self._restart_mirror, args=[msg, start_time])
            thread.daemon = True
            thread.start()

            # Force a new update after sync
            Logger.localinfo('\x1b[93mReceived sync for current behavior - request behavior update message\x1b[0m')
            MirrorStateMachine._execute_flag = True  # Execute once more after any change,
            sm._last_deep_states_list = None

        else:
            self._log_exception('Mirror synchronize request mismatch',
                                SyncError(f'id={msg.behavior_id} mismatch active={active_id}'),
                                start_time=start_time)
            if self._running:
                Logger.localinfo('    stopping current mirror')
                thread = threading.Thread(target=self._stop_mirror, args=[None, start_time])
                thread.daemon = True
                thread.start()

            if self._last_obe_status in (BEStatus.RUNNING, BEStatus.STARTED, BEStatus.SWITCHING):
                if not self._starting:
                    Logger.logwarn(f'OBE is {bestatus_map[self._last_obe_status]}, '
                                   'so request current mirror structure from onboard ...')
                    self._request_struct_pub.publish(Int32(data=msg.behavior_id))
                    self._starting = True
                    self._notify_state_change()
                else:
                    Logger.localwarn(f'OBE is {bestatus_map[self._last_obe_status]}, '
                                     'but we have already re-requested mirror structure from onboard!')

    def _onboard_heartbeat_callback(self, msg):
        """Call in response to onboard heartbeat."""
        try:
            """Use heartbeat to monitor for persistent sync issues."""
            if self._active_id == BehaviorSync.INVALID:
                return  # do not check sync status if no behavior is active

            if msg.behavior_id == self._active_id:
                if self._sm is not None:
                    # This is where we want to be
                    mirror_status = self._sm.get_latest_status()
                    if mirror_status.behavior_id != self._active_id:
                        Logger.localwarn_throttle(
                            0.5,
                            'mirror_status.behavior_id (%s) != self._active_id (%s)',
                            mirror_status.behavior_id,
                            self._active_id,
                        )
                    mirror_status.behavior_id = self._active_id
                    onboard_sig = tuple(msg.current_state_checksums)
                    mirror_sig = tuple(mirror_status.current_state_checksums)

                    if onboard_sig != mirror_sig:
                        previous_sig = (
                            self._last_onboard_mismatch_sig,
                            self._last_mirror_mismatch_sig,
                        )
                        current_sig = (onboard_sig, mirror_sig)
                        same_mismatch = current_sig == previous_sig
                        self._sync_heartbeat_mismatch_counter = (
                            self._sync_heartbeat_mismatch_counter + 1 if same_mismatch else 1
                        )
                        onboard_state_path = 'Unknown'
                        if same_mismatch and self._sync_heartbeat_mismatch_counter > 1:
                            # Two consecutive matching out-of-sync heartbeats.
                            if len(onboard_sig) > 0:
                                # Use deepest state as the best path estimate
                                ob_state_id, ob_out = StateMap.unhash(onboard_sig[-1])
                                ob_state = self._state_map.get_state(ob_state_id)
                                if ob_state is not None:
                                    onboard_state_path = ob_state.path

                        if self._sync_heartbeat_mismatch_counter == 1:
                            # Single-heartbeat divergences are common around rapid transitions.
                            # Wait for the next heartbeat to confirm the same mismatch signature.
                            self._last_onboard_mismatch_sig = onboard_sig
                            self._last_mirror_mismatch_sig = mirror_sig
                            Logger.localinfo_throttle(
                                0.5,
                                'Transient OCS heartbeat divergence detected for behavior %s; '
                                'waiting for confirmation of the same mismatch on the next heartbeat.',
                                self._active_id,
                            )
                        elif self._sync_heartbeat_mismatch_counter == 2:
                            # Confirmed repeated mismatch signature.
                            self._set_sync_warning_state(True, behavior_id=self._active_id)
                            Logger.localerr(
                                'OCS is possibly out of sync - onboard state is %s\n'
                                '    Check UI and consider manual re-sync!\n'
                                '    (mismatch may be temporarily understandable for rapidly changing outcomes) %s',
                                onboard_state_path,
                                self._sync_heartbeat_mismatch_counter,
                            )
                            self._log_compact_sync_summary(onboard_sig, mirror_sig)
                            if self._sync_heartbeat_mismatch_counter % 20 == 1:
                                Logger.info('Verify sync with onboard.')  # Message to OCS
                        elif self._sync_heartbeat_mismatch_counter % 10 == 1:
                            self._log_compact_sync_summary(onboard_sig, mirror_sig)
                    else:
                        self._last_onboard_mismatch_sig = None
                        self._last_mirror_mismatch_sig = None
                        self._set_sync_warning_state(False, behavior_id=self._active_id)
                        if self._sync_heartbeat_mismatch_counter > 1:
                            Logger.localwarn(f'OCS is back in sync after {self._sync_heartbeat_mismatch_counter} heartbeats')
                            Logger.localinfo(
                                'Recovered sync for IDs %s/%s with onboard=%s mirror=%s',
                                msg.behavior_id,
                                self._active_id,
                                onboard_sig,
                                mirror_sig,
                            )

                        # Reset mismatch counter
                        self._sync_heartbeat_mismatch_counter = 0
                elif self._active_id != 0:
                    Logger.warning(f'Received matching behavior id {msg.behavior_id} with no mirror state machine active!')
                else:
                    Logger.warning(f'Received invalid behavior id {msg.behavior_id} with active id = {self._active_id} active!')

            elif msg.INVALID not in (msg.behavior_id, self._active_id):
                if self._sync_heartbeat_mismatch_counter == 0:
                    self._set_sync_warning_state(True, behavior_id=self._active_id)
                if self._sync_heartbeat_mismatch_counter % 10 == 1:
                    Logger.error('Out of sync! Different behavior is running onboard, please stop execution! '
                                 f'{self._sync_heartbeat_mismatch_counter}')
                self._sync_heartbeat_mismatch_counter += 1
            elif not self._stopping:
                if self._sync_heartbeat_mismatch_counter == 0:
                    self._set_sync_warning_state(True, behavior_id=self._active_id)
                self._sync_heartbeat_mismatch_counter += 1
                if self._sync_heartbeat_mismatch_counter % 10 == 1:
                    Logger.warning(f"Mismatched behavior ids ('{msg.behavior_id}', '{self._active_id}')- "
                                   f'please restart behavior! {self._sync_heartbeat_mismatch_counter}')
            else:
                Logger.localinfo(f'Heartbeat: mirror is stopping - waiting for  {self._active_id} to stop ...')

        except Exception as exc:
            # TODO: High-risk guardrail. Heartbeat must never crash; narrow this catch once mismatch handling is isolated.
            self._log_exception('Exception in heartbeat callback', exc, default_status=BEStatus.WARNING)

    def _wait_stop_running(self, start_time):
        """Send stop command to mirror SM and wait for running mirror thread to stop."""
        if self._running:
            PreemptableState.preempt = True
            self._stopping = True
            self._notify_state_change()
            running_cnt = 0
            timeout_sec = self._wait_timeout_sec
            polling_sec = self._wait_poll_sec
            deadline = time.monotonic() + timeout_sec
            while self._running:
                if running_cnt % 100 == 0:
                    try:
                        Logger.localinfo(f'Waiting for another mirror (start thread {self._active_thread_start}) to stop '
                                         f"with id = '{self._active_id}' "
                                         f'(this {self.get_elapsed_str(start_time)}) (running cnt={running_cnt}) '
                                         f'preempt={PreemptableState.preempt}')
                    except (AttributeError, RuntimeError):
                        # Likely during shutdown
                        print(f'Waiting for another mirror (start thread {self._active_thread_start}) to stop '
                              f"with id = '{self._active_id}' "
                              f'(this {self.get_elapsed_str(start_time)}) (running cnt={running_cnt})', flush=True)
                running_cnt += 1
                if time.monotonic() >= deadline:
                    Logger.logerr(f'Timeout waiting for another mirror ({self._active_thread_start}) to stop running '
                                  f' with {self._active_id} (this {self.get_elapsed_str(start_time)})')
                    # Recovery path: avoid leaving mirror permanently latched in stopping mode.
                    self._stopping = False
                    self._start_requested = False
                    self._starting = False
                    self._notify_state_change()
                    raise SyncError(f"Timeout waiting for mirror '{self._active_id}' to stop running")

                self._timing_event.clear()
                if not self._running:
                    break
                # Release lock while waiting so _execute_mirror's finally block can acquire it to clear _running.
                self._sync_lock.release()
                try:
                    self._timing_event.wait(polling_sec)  # Prefer state-change wakeups and fall back to timeout.
                finally:
                    self._sync_lock.acquire()
            Logger.localinfo(f'Mirror for active id {self._active_id} stopped running (start thread {self._active_thread_start}) '
                             f' ({running_cnt}) (this {self.get_elapsed_str(start_time)})')
            Logger.loginfo('Mirror stopped running!')
            self._stopping = False  # We are done stopping with success
            self._start_requested = False
            self._starting = False  # Clear for next structure request
            self._notify_state_change()

    def _wait_stopping(self, start_time):
        """Wait for stop command to complete."""
        if self._stopping:
            stopping_cnt = 0
            timeout_sec = self._wait_timeout_sec
            polling_sec = self._wait_poll_sec
            deadline = time.monotonic() + timeout_sec
            while self._stopping:
                if stopping_cnt % 100 == 0:
                    try:
                        Logger.localinfo(f'Waiting for another mirror (start thread {self._active_thread_start}) '
                                         f" to finish stopping  with id='{self._active_id}' "
                                         f'(this {self.get_elapsed_str(start_time)}) (stopping {stopping_cnt})... ')
                    except (AttributeError, RuntimeError):
                        print(f'Waiting for another mirror (start thread {self._active_thread_start}) '
                              f" to finish stopping  with id='{self._active_id}' "
                              f'(this {self.get_elapsed_str(start_time)}) (stopping {stopping_cnt})... ', flush=True)
                stopping_cnt += 1
                if time.monotonic() >= deadline:
                    Logger.logerr(f"Timeout waiting for another mirror to finish stopping with '{self._active_id}'"
                                  f' (this {self.get_elapsed_str(start_time)})')
                    # Recovery path: clear stale stop flags so future start/stop requests can proceed.
                    self._stopping = False
                    self._start_requested = False
                    self._starting = False
                    self._notify_state_change()
                    raise SyncError(f"Timeout waiting for mirror '{self._active_id}' to finish stopping")

                self._timing_event.clear()
                if not self._stopping:
                    break
                # Release lock while waiting so threads clearing _stopping can acquire it without deadlock.
                self._sync_lock.release()
                try:
                    self._timing_event.wait(polling_sec)  # Prefer state-change wakeups and fall back to timeout.
                finally:
                    self._sync_lock.acquire()
            Logger.localinfo(f'Mirror completed stopping for active id {self._active_id} '
                             f' ({self._starting}, {self._running}, {self._stopping})'
                             f' (this {self.get_elapsed_str(start_time)}) ({stopping_cnt})!')
            Logger.loginfo('Mirror stopped running.')
            self._start_requested = False
            self._starting = False  # Clear for next structure request
            self._notify_state_change()

    def _reinitialize_state_machine(self, state_machine):
        """
        Reinitialize existing SM when restarting mirror during sync.

        This is significantly faster than rebuilding current structure.
        """
        state_machine._entering = False
        state_machine._current_state = None
        for state in state_machine._states:
            state._entering = True
            if isinstance(state, MirrorStateMachine):
                self._reinitialize_state_machine(state)

    def _restart_mirror(self, msg, restart_time):
        """Restart mirror to force resync with onboard."""
        set_thread_name('rsm' + f'{restart_time.nanoseconds}'[-12:])  # only 15 chars allowed
        with self._sync_lock:
            try:
                if self._sm is not None and self._running:
                    self._wait_stop_running(restart_time)

                Logger.localinfo(f'Restarting mirror for synchronization of behavior checksum id ={msg.behavior_id} '
                                 f'with active id={self._active_id}')

                # Clear existing outcome messages
                self._outcome_sub.remove_last_msg(Topics._OUTCOME_TOPIC, clear_buffer=True)
                MirrorState._last_state_id = None
                MirrorState._last_state_outcome = None
                MirrorState._last_target_id = None  # reset any time that we build a new state machine

                self._starting_path = None
                if self._sm is not None and self._sm.id == msg.behavior_id:
                    PreemptableState.preempt = False  # Reset preempt flag before restarting
                    self._reinitialize_state_machine(self._sm)
                    Logger.localinfo(f'Done reinitializing the existing state machine with matching '
                                     f"behavior id='{msg.behavior_id}' ")
                else:
                    if self._current_struct is not None and self._current_struct.behavior_id == msg.behavior_id:
                        # Reconstruct the state machine from existing structure
                        self._mirror_state_machine(self._current_struct)
                    elif self._sm is not None:
                        # Not running, but not the correct SM either
                        Logger.localinfo(f'Discard mismatched behavior SM={self._sm.id} '
                                         f"vs. requested behavior id='{msg.behavior_id}'")
                        self._sm = None

                    if self._sm is None:
                        if not self._starting:
                            Logger.logwarn('Requesting mirror structure from onboard ...')
                            self._request_struct_pub.publish(Int32(data=msg.behavior_id))
                        else:
                            Logger.localwarn('Invalid SM cannot restart - mirror structure previously requested from onboard!')
                        return

                assert self._sm.id == msg.behavior_id, ('Error in restart_mirror: '
                                                        f'mismatched behavior SM={self._sm.id} '
                                                        f'vs. requested behavior id={msg.behavior_id}')
                self._clear_terminal_status()

                Logger.localinfo(f' Reset active states using current state checksums: {msg.current_state_checksums}')
                for active_state in msg.current_state_checksums:
                    # Heartbeat snapshots now encode "no outcome" distinctly as None, so a
                    # non-None value really does represent a pending outcome to replay.
                    state_id, outcome = StateMap.unhash(active_state)
                    state = self._state_map[state_id]
                    if state is not None:
                        if self._starting_path is None:
                            self._starting_path = state.path

                        if outcome is not None and MirrorState._last_state_id is None:
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
                                Logger.logerr(f"            Sync: Unexpected parent reference '{parent.name}' ({type(parent)}) "
                                              f"from '{state.name}' in '{self._sm.name}'")
                            state = parent
                            parent = parent.parent
                    else:
                        Logger.logerr(f"        Unknown state from {state_id} in '{self._sm.name}' from restart "
                                      f'in thread {self.get_elapsed_str(restart_time)}!')
                curst = self._sm._current_state
                self._sm._last_deep_states_list = self._sm.get_deep_states()
                Logger.localwarn(f" Restart SM with current top-level state = {curst.name if curst is not None else 'None'} "
                                 f'starting path={self._starting_path}')
                Logger.localinfo(f'     active states = {self._sm.get_latest_status()}')
                if self._sm._last_deep_states_list is not None and len(self._sm._last_deep_states_list) > 0:
                    # Make sure we update the UI with latest state
                    MirrorState.publish_update(self._sm._last_deep_states_list[-1].state_id)
                    for st in self._sm._last_deep_states_list:
                        Logger.localinfo(f"     '{st.name:30s}' - '{st.path}' ")
                else:
                    MirrorState.publish_update(curst.state_id)  # Make sure we update the UI with latest state
                self._running = True  # set running while we have sync lock
                self._starting = False
                self._active_id = msg.behavior_id
                self._notify_state_change()
            except (AttributeError, RuntimeError, SyncError, TransitionError) as exc:
                Logger.loginfo(f'Stopping synchronization because behavior{msg.behavior_id} has stopped.')
                Logger.localinfo(f"'{type(exc)}' - {exc}")
                return

        try:
            self._execute_mirror(restart_time)
        except (SyncError, TransitionError) as exc:
            self._handle_execution_exception('Exception in restart_mirror', exc, restart_time)
        except Exception as exc:  # pylint: disable=W0703
            # TODO: High-risk guardrail. Revisit narrowing after proving no lifecycle regressions during restart/stop races.
            self._handle_execution_exception('Exception in restart_mirror', exc, restart_time)

        Logger.localwarn(f'Finished execution of restart request for behavior checksum id ={msg.behavior_id} '
                         f'in thread {self.get_elapsed_str(restart_time)}.')

    def _execute_mirror(self, start_time):
        """Run the mirrored SM structure and follow transitions to stay in sync with OBE."""
        Logger.localinfo(f'Execute mirror behavior id ={self._active_id} ({self._sm.id})')
        if self._active_thread_start is not None:
            Logger.localwarn(f'Trying to start execution for {start_time.nanoseconds} but '
                             f'older {self._active_thread_start} is still active!')
            PreemptableState.preempt = True
            raise TransitionError('Mirror issue - shutdown all threads - need to reattach!')

        # Callers set self._running = True under _sync_lock before calling here;
        # do not set it again so that flag ownership stays with the caller.
        if PreemptableState.preempt:
            # A stop was requested during the window between lock release and spin start.
            # Abort cleanly and reset preempt so the next execution is not poisoned.
            Logger.localwarn('Mirror preempt set before execution started; aborting execute.')
            PreemptableState.preempt = False
            with self._sync_lock:
                self._running = False
            self._notify_state_change()
            return

        self._active_thread_start = start_time.nanoseconds  # Track starting time
        self._notify_state_change()
        Logger.loginfo('Executing mirror ...')
        Logger.localinfo(f'  in thread {self.get_elapsed_str(start_time)} s ')
        if self._starting_path is not None:
            LockableStateMachine.path_for_switch = self._starting_path
            Logger.loginfo('Starting mirror in state ' + self._starting_path)
            self._starting_path = None

        result = State._preempted_name
        try:
            result = self._sm.spin(start_time, self._state_map)
            Logger.localinfo(f"Mirror finished spin with result '{result}' after {self.get_elapsed_str(start_time)} s")
            self._sm.destroy()
            self._sm = None  # Clear reference so _stop_mirror knows execution already completed
        except (SyncError, TransitionError):
            raise
        except Exception as exc:
            # TODO: High-risk guardrail. Keep broad catch while preserving preempt/recovery behavior under executor teardown.
            try:
                Logger.logerr('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc))
                Logger.localerr(traceback.format_exc().replace('%', '%%'))
            except (AttributeError, RuntimeError):
                # Likely the loggers are dead if we ctrl-C'd during active behavior
                # so just try a simple print
                print('\n(_execute_mirror Traceback): Caught exception on preempt:\n%s' % str(exc), flush=True)
                print(traceback.format_exc().replace('%', '%%'), flush=True)
            result = State._preempted_name
        finally:
            self._active_thread_start = None
            with self._sync_lock:
                self._running = False
            self._notify_state_change()

    def _mirror_state_machine(self, msg):
        """Construct mirror of onboard state machine given structure."""
        start = time.time()  # Track construction time
        try:
            self._current_struct = msg
            self._state_map = StateMap()
            self._last_onboard_mismatch_sig = None
            self._last_mirror_mismatch_sig = None
            root, structure_index = self._index_structure(msg)

            # self.get_logger().info(f'Constructing top-level mirror for {root} ...')
            self._add_node(root, structure_index)

            if self._sm:
                if self._sm.name is None:
                    self._sm.set_name(root or 'root')
                Logger.localinfo('---------------------------------')
                self._sm.id = msg.behavior_id
                self._sm._status_event_callback = self._mirror_transition_callback
                self._sm._outcome_sub = self._outcome_sub
                Logger.localinfo(f"Constructed mirror for behavior id ='{self._sm.id}' - begin validation ...")
                # verify checksums of all states
                for con_msg in msg.containers:
                    if con_msg.path.find('/') != -1:
                        state = self._state_map.get_state(con_msg.state_id)
                        if state:
                            expected_path = structure_index['mirror_paths_by_id'][con_msg.state_id]
                            assert state.path == expected_path, (
                                f'Mismatched state id={state.state_id} for {state.path} vs. '
                                f'({con_msg.state_id}) {con_msg.path}'
                            )
                        else:
                            raise KeyError(f'State id {con_msg.state_id} not found in {self._state_map}!')

                try:
                    state_ids, state_paths = list(zip(*self._state_map.items))
                    state_map_msg = StateMapMsg(behavior_id=self._sm.id,
                                                state_ids=state_ids,
                                                state_paths=[
                                                    structure_index['display_paths_by_id'][state_id]
                                                    for state_id, _path in zip(state_ids, state_paths)
                                                ])
                    self._state_map_pub.publish(state_map_msg)  # Used by the WebUI
                except (AttributeError, TypeError, ValueError) as exc:
                    Logger.localerr(f'Failed to publish state map: {exc}')

                end = time.time()
                Logger.localinfo(f"Validated constructed mirror for behavior id ='{self._sm.id}' in {end - start} seconds !")
                return  # success here
            else:
                Logger.logerr(f'Failed to construct mirror SM for {root}!')

        except Exception as exc:
            # TODO: High-risk guardrail. Keep broad catch to avoid partial mirror graph leakage on malformed structures.
            Logger.localwarn(f"_mirror_statemachine Exception: '{type(exc)}' - {exc}")
            if self._sm is not None:
                Logger.localwarn(f'    destroy constructed SM id={self._sm.id} - failed validation!')
                self._sm.destroy()
            self._sm = None

        end = time.time()
        Logger.localinfo(f"Failed to construct mirror of state machine '{msg.behavior_id}' in {end - start} seconds !")

    @staticmethod
    def _expected_mirror_path(path):
        """Convert an onboard structure path into the corresponding mirror path."""
        fragments = [frag for frag in path.split('/') if frag]
        mirror_path = '/'.join(f'{frag}_mirror' for frag in fragments)
        return f'/{mirror_path}' if path.startswith('/') else mirror_path

    @staticmethod
    def _index_structure(msg):
        """Index structure containers once for efficient mirror reconstruction."""
        structure_index = {
            'containers_by_path': {},
            'child_paths_by_path': {},
            'container_names_by_path': {},
            'mirror_paths_by_id': {},
            'display_paths_by_id': {},
        }
        root = None

        for container in msg.containers:
            structure_index['containers_by_path'][container.path] = container
            structure_index['child_paths_by_path'][container.path] = [
                f'{container.path}/{child}' for child in container.children
            ]
            structure_index['container_names_by_path'][container.path] = container.path.rsplit('/', 1)[-1]
            structure_index['mirror_paths_by_id'][container.state_id] = FlexbeMirror._expected_mirror_path(container.path)
            structure_index['display_paths_by_id'][container.state_id] = container.path
            if root is None and container.path.find('/') == -1:
                root = container.path

        if root is None:
            raise KeyError('Unable to identify top-level container in structure message')

        return root, structure_index

    def _add_node(self, path, structure_index):
        """Add node (state) to state machine graph based on type."""
        container = structure_index['containers_by_path'][path]

        transitions = None
        if container.transitions is not None:
            transitions = {}
            for i in range(len(container.transitions)):
                transitions[container.outcomes[i]] = container.transitions[i] + '_mirror'

        container_name = structure_index['container_names_by_path'][path]

        if len(container.children) > 0:
            sm_outcomes = []
            for outcome in container.outcomes:
                sm_outcomes.append(outcome + '_mirror')
            if container.type == OperatableStateMachine.ContainerType.ConcurrencyContainer.value:
                sm = MirrorConcurrencyContainer(container_name, path, outcomes=sm_outcomes)
            elif container.type == OperatableStateMachine.ContainerType.PriorityContainer.value:
                sm = MirrorPriorityContainer(container_name, path, outcomes=sm_outcomes)
            else:
                sm = MirrorStateMachine(container_name, path, outcomes=sm_outcomes)

            self._state_map.add_state(path, sm)  # also calculates the state id given path
            assert sm.state_id == container.state_id, ('Failed to validate container state_id '
                                                       f'= {sm.state_id} vs. {container.state_id}')

            with sm:
                for child_path in structure_index['child_paths_by_path'][path]:
                    self._add_node(child_path, structure_index)
            if len(transitions) > 0:
                container_transitions = {}
                for i in range(len(container.transitions)):
                    container_transitions[sm_outcomes[i]] = transitions[container.outcomes[i]]
                MirrorStateMachine.add(container_name + '_mirror', sm, transitions=container_transitions)
            else:
                # Add instance attributes to top-level state machine
                sm._total_loop_count = 0
                self._sm = sm

        else:
            # Basic state
            assert container.type == 0, f"'{container_name}' - Non-containers should have type 0 not {container.type}!"
            mrst = MirrorState(container_name, path, container.outcomes, container.autonomy)
            self._state_map.add_state(path, mrst)
            assert mrst.state_id == container.state_id, ('Failed to validate container state_id '
                                                         f'= {mrst.state_id} vs. {container.state_id}')
            MirrorStateMachine.add(container_name + '_mirror', mrst, transitions=transitions)

    def _preempt_callback(self, msg):
        """Call on receipt of preempt mirror message (ignored)."""
        # pylint: disable=unused-argument
        if self._sm is not None:
            Logger.logwarn('Explicit preempting is currently ignored, mirror should be preempted by onboard behavior.')
        else:
            Logger.logwarn('Could not preempt mirror because it seems not to be running!')
