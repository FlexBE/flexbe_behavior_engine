#!/usr/bin/env python3

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
#    * Neither the name of the Philipp Schillinger, Team ViGIR,
#      Christopher Newport University nor the names of its
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


"""Class defining state machine executive for onboard control."""

import contextlib
import os
import shutil
import sys
import tempfile
import threading
import time
import zlib
from ast import literal_eval as cast
from datetime import datetime

try:
    from prctl import set_name as set_thread_name
except ImportError:
    def set_thread_name(name):
        """Set thread name if prctl is not available."""
        print('Python thread names are not visible in ps/top unless you install prctl')

from flexbe_core import BehaviorLibrary, Logger, MIN_UI_VERSION
from flexbe_core.core import BehaviorLoadError, ProxyError, ShutdownError, SyncError, TransitionError
from flexbe_core.core import map_exception_to_bestatus
from flexbe_core.core.state_machine import StateMachine
from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_core.proxy.qos import QOS_OUTCOME

from flexbe_msgs.msg import BEStatus, BehaviorSelection, BehaviorSync, CommandFeedback, StateMapMsg, UserdataInfo
from flexbe_msgs.srv import GetUserdata

import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import String, UInt32


class FlexbeOnboard(Node):
    """Control the execution of robot behaviors."""

    def __init__(self):
        """Initiate the Node class's constructor and give it a name."""
        super().__init__('flexbe_onboard')

        ProxyPublisher.initialize(self)
        ProxySubscriberCached.initialize(self)
        Logger.initialize(self)

        self.be = None
        self.executor = None
        self._tracked_imports = []
        # prepare temp folder
        self._tmp_folder = tempfile.mkdtemp()
        sys.path.append(self._tmp_folder)

        # prepare manifest folder access
        self._behavior_lib = BehaviorLibrary(self)

        # prepare communication
        # Proxy as also accessed by states
        self._proxy_pub = ProxyPublisher({
            Topics._CMD_FEEDBACK_TOPIC: CommandFeedback,
        })
        self._proxy_pub.create_publisher(Topics._OUTCOME_TOPIC, UInt32, qos=QOS_OUTCOME)

        # only at onboard level
        self._heartbeat_pub = self.create_publisher(BehaviorSync, Topics._ONBOARD_HEARTBEAT_TOPIC, 10)
        self._idle_heartbeat = BehaviorSync()
        self._ready_status = BEStatus(code=BEStatus.READY)

        # Latch state map so we can retrieve later if desired
        # Keep enough transient-local history for reconnecting consumers to observe
        # the full terminal/startup sequence across rapid restarts and switches.
        latching_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(BEStatus, Topics._ONBOARD_STATUS_TOPIC, qos_profile=latching_qos)
        self._state_map_pub = self.create_publisher(StateMapMsg, Topics._STATE_MAP_TOPIC, qos_profile=latching_qos)

        self._version_sub = self.create_subscription(String, Topics._UI_VERSION_TOPIC,
                                                     self._version_callback, qos_profile=latching_qos)

        # listen for new behavior to start
        self._start_beh_sub = self.create_subscription(BehaviorSelection,
                                                       Topics._START_BEHAVIOR_TOPIC,
                                                       self._behavior_callback,
                                                       10)

        try:
            self._enable_clear_imports = self.get_parameter('enable_clear_imports').get_parameter_value()
        except ParameterNotDeclaredException:
            self.declare_parameter('enable_clear_imports', False)
            self._enable_clear_imports = self.get_parameter('enable_clear_imports').get_parameter_value()

        self._trigger_ready = True
        self._running = False
        self._run_lock = threading.Lock()
        self._run_state_event = threading.Event()
        self._starting = False
        self._start_lock = threading.Lock()  # Protect _starting check-then-set in _behavior_callback
        self._switching = False
        self._switch_lock = threading.Lock()
        self._behavior_id = -1
        self._current_state_checksum = -1

        self._userdata_service = self.create_service(GetUserdata, 'get_user_data', self._userdata_callback)

        time.sleep(0.1)  # wait for publishers etc to really be set up

        # Will also re-publish BEStatus.READY every 10 seconds until first behavior received
        Logger.localinfo('Set up the heartbeat timer ...')
        self._trigger_ready = False
        self._ready_counter = 0
        self._heartbeat = self.create_timer(1.0, self._heartbeat_worker)

        Logger.localinfo('\033[92m--- Behavior Engine ready for first behavior! ---\033[0m')
        self._publish_ready_status()

    def _version_callback(self, msg):
        """Check required version of UI."""
        vui = FlexbeOnboard._parse_version(msg.data)
        vex = FlexbeOnboard._parse_version(MIN_UI_VERSION)
        if vui < vex:
            Logger.logwarn('FlexBE UI needs to be updated!\n'
                           f'Onboard Behavior Engine requires at least version {MIN_UI_VERSION}, '
                           f' but you have {msg.data}\n'
                           'Please update the FlexBE UI software.')

    @staticmethod
    def _parse_version(v):
        """Parse the UI version string."""
        result = 0
        offset = 1
        for n in reversed(v.split('.')):
            result += int(n) * offset
            offset *= 100
        return result

    def _behavior_callback(self, beh_sel_msg):
        with self._start_lock:
            if self._starting:
                # Prevent multiple request messages from triggering too soon
                Logger.logwarn_throttle(
                    2.0,
                    'Received behavior start request for %s (%s) while prior request was '
                    'starting.\n    Ignore second request!',
                    beh_sel_msg.behavior_key,
                    beh_sel_msg.behavior_id
                )
                return
            self._starting = True  # Prevent two start requests from occurring back to back
        self._trigger_ready = False  # We have received the behavior selection request
        self._ready_counter = 0
        thread = threading.Thread(target=self._behavior_execution, args=[beh_sel_msg])
        thread.daemon = True
        thread.start()

    def behavior_shutdown(self):
        """Destroy any active behavior state machines to force proper shutdown."""
        try:
            print(f'    Shutting down onboard behavior engine at {datetime.now()} ...', flush=True)
            with self._switch_lock:
                if self._running:
                    assert self.be is not None, 'Must have an active behavior here!'
                    self._switching = True
                    self.be.preempt()

                    print('    Waiting for existing behavior to terminate ...', flush=True)
                    return True  # Active behavior needs to quit

            return False  # No active behavior

        except (AssertionError, AttributeError, RuntimeError) as exc:
            print(f"Exception shutting down onboard behaviors '{type(exc)}'\n   {exc}", flush=True)
            import traceback
            print(traceback.format_exc().replace('%', '%%'), flush=True)

    def onboard_shutdown(self):
        """Shutdown the onboard software."""
        print('Shutting down the onboard behavior executive ...', flush=True)
        self.destroy_timer(self._heartbeat)
        self._proxy_pub.remove_publisher(Topics._CMD_FEEDBACK_TOPIC)
        if self.executor:
            for _ in range(50):
                self.executor.spin_once(timeout_sec=0.001)

    def verify_no_active_behaviors(self, timeout=0.5):
        """Verify no active behaviors."""
        run_locked = self._run_lock.acquire(timeout=timeout)
        if run_locked:
            try:
                assert self.be is None, 'Run lock with old behavior active?'
                self._switching = False
                print(f'    All onboard behaviors are stopped at {datetime.now()}!', flush=True)
                return True
            finally:
                self._run_lock.release()
        else:
            return False

    # =================== #
    # Main execution loop #
    # ------------------- #

    def _behavior_execution(self, beh_sel_msg):
        # sending a behavior while one is already running is considered as switching
        set_thread_name(f'beh{beh_sel_msg.behavior_id}')
        if not rclpy.ok():
            self._cleanup_tempdir()
            self._starting = False
            self._switching = False
            return

        if self._running:
            Logger.loginfo('--> Initiating behavior switch...')
            self._proxy_pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='switch', args=['received']))

        # construct the behavior that should be executed
        Logger.localinfo(f'Prepare behavior id={beh_sel_msg.behavior_key} ({beh_sel_msg.behavior_id}) ...')
        be = self._prepare_behavior(beh_sel_msg)
        if be is None:
            Logger.logerr('Dropped behavior start request because preparation failed.')
            if self._running:
                self._proxy_pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='switch', args=['failed']))
            else:
                # self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(), code=BEStatus.READY))
                Logger.localinfo('\033[92m--- Behavior Engine ready to try again! ---\033[0m')
                self._ready_counter = 8  # Trigger heartbeat to republish READY within 2 seconds
            self._starting = False  # Now allow a new start requests
            return

        # perform the behavior switch if required
        # Logger.localinfo("Behavior Engine - get switch lock to start behavior id "
        #                  f"key={beh_sel_msg.behavior_key} ({beh_sel_msg.behavior_id})...")
        with self._switch_lock:
            Logger.localinfo('Behavior Engine - got switch lock to start behavior new id '
                             f'key={beh_sel_msg.behavior_key} ({beh_sel_msg.behavior_id})...')
            if self._running:
                assert self.be is not None, 'Must have an active behavior here!'
                self._switching = True
                Logger.localinfo('Behavior Engine - prepare to switch current running behavior'
                                 f" '{self.be.name}': id={self.be.beh_id}...")
                self._proxy_pub.publish(Topics._CMD_FEEDBACK_TOPIC, CommandFeedback(command='switch', args=['start']))

                # ensure that switching is possible
                if not self._is_switchable(be):
                    Logger.logerr('Dropped behavior start request for '
                                  f'key={beh_sel_msg.behavior_key} (id={beh_sel_msg.behavior_id}) '
                                  ' because switching is not possible.')
                    self._proxy_pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                            CommandFeedback(command='switch', args=['not_switchable']))
                    if self._enable_clear_imports:
                        self._clear_imports()
                    self._cleanup_behavior(beh_sel_msg.behavior_id)
                    self._switching = False
                    self._starting = False  # Now allow a new start requests
                    return

                self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                                  behavior_id=self.be.beh_id,
                                                  code=BEStatus.SWITCHING))
                # Wait for the current behavior to expose active states or stop running.
                active_states = None
                active_leaf_states = ()
                while rclpy.ok() and self._running:
                    active_states = self.be.get_current_states()
                    active_leaf_states = self._get_switch_leaf_states(active_states)
                    if not self._running or len(active_leaf_states) > 0:
                        break
                    self._run_state_event.clear()
                    active_states = self.be.get_current_states()
                    active_leaf_states = self._get_switch_leaf_states(active_states)
                    if not self._running or len(active_leaf_states) > 0:
                        break
                    self._run_state_event.wait(0.05)

                # extract the active state if any
                if len(active_leaf_states) > 0:
                    Logger.localinfo(f"Behavior Engine - '{be.name}': {be.beh_id} "
                                     f'switching behaviors from active state {[acst.name for acst in active_states]} ...')
                    try:
                        if len(active_leaf_states) > 1:
                            leaf_names = [state.name for state in active_leaf_states]
                            Logger.logwarn(f"Cannot switch behavior '{be.name}' with multiple active leaf states: {leaf_names}")
                            self._proxy_pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                                    CommandFeedback(command='switch', args=['not_switchable']))
                            if self._enable_clear_imports:
                                self._clear_imports()
                            self._cleanup_behavior(beh_sel_msg.behavior_id)
                            self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                                              behavior_id=self.be.beh_id,
                                                              code=BEStatus.RUNNING))
                            self._switching = False
                            self._starting = False  # Now allow a new start requests
                            return

                        active_state = active_leaf_states[0]
                        be.prepare_for_switch(active_state)
                        self._proxy_pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                                CommandFeedback(command='switch', args=['prepared']))
                    except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
                        Logger.logerr('Failed to prepare behavior switch:\n%s' % str(exc))
                        self._proxy_pub.publish(Topics._CMD_FEEDBACK_TOPIC,
                                                CommandFeedback(command='switch', args=['failed']))
                        if self._enable_clear_imports:
                            self._clear_imports()
                        self._cleanup_behavior(beh_sel_msg.behavior_id)
                        # Let us know that old behavior is still running
                        self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                                          behavior_id=self.be.beh_id,
                                                          code=BEStatus.RUNNING))
                        self._switching = False
                        self._starting = False  # Now allow a new start requests
                        return
                    # stop the rest
                    Logger.localinfo(f"Behavior Engine - '{self.be.name}': {self.be.beh_id} - "
                                     f"preempt active state  '{active_state.name}' ...")
                    self.be.preempt()
                else:
                    Logger.localinfo(f"Behavior Engine - '{self.be.name}': {self.be.beh_id} "
                                     f'no active leaf state to preempt (but Running={self._running}?)!')

        # execute the behavior
        Logger.localinfo('Waiting on prior behavior to shutdown ...')
        with self._run_lock:
            Logger.localinfo('Behavior Engine - got run lock to start behavior id '
                             f'key={beh_sel_msg.behavior_key}={be.beh_id} ({beh_sel_msg.behavior_id}) ...')
            assert self.be is None, 'Run lock with old behavior active?'
            self._running = True
            self._starting = False  # Now allow a new start requests
            self.be = be
            self._run_state_event.set()

            result = None
            try:
                Logger.loginfo('Onboard Behavior Engine starting [%s : %s]' % (be.name, beh_sel_msg.behavior_id))
                be.confirm()
                self._run_state_event.set()
                Logger.localinfo(f'    behavior {be.name}: {be.beh_id} confirmation.')

                # Publish behavior state map as a debugging aid (match to OCS side published by launcher and mirror)
                state_ids, state_paths = be.state_map_items
                state_map_msg = StateMapMsg(behavior_id=be.beh_id,
                                            state_ids=state_ids,
                                            state_paths=state_paths)
                self._state_map_pub.publish(state_map_msg)

                # Publish start status
                args = [be.requested_state_id] if be.requested_state_id is not None else []
                Logger.localinfo(f'Behavior Engine - behavior {be.name}: {be.beh_id} BEStatus STARTED.')
                self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                                  behavior_id=be.beh_id,
                                                  code=BEStatus.STARTED,
                                                  args=args))

                # Do the behavior
                Logger.localinfo(f'Behavior Engine - behavior {be.name}: {be.beh_id} begin execution ...')
                result = be.execute()

                Logger.localinfo(f'Behavior Engine - {be.name}: {be.beh_id} done execute with result={result}')
                try:
                    self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                                      behavior_id=be.beh_id,
                                                      code=BEStatus.FINISHED,
                                                      args=[str(result)]))
                except (InvalidHandle, RuntimeError):
                    pass  # Publisher destroyed during teardown; behavior did finish successfully.
            except (BehaviorLoadError, ProxyError, ShutdownError, SyncError, TransitionError) as exc:
                result = self._report_execution_failure(exc, result)
            except Exception as exc:
                # TODO: High-risk guardrail.
                # Keep broad catch to prevent behavior thread crashes; narrow after execution-path audit.
                result = self._report_execution_failure(exc, result)
            finally:
                # Release self.be immediately so the heartbeat reverts to idle while cleanup runs.
                # The local `be` variable retains the reference for cleanup below.
                self.be = None

            # done, remove left-overs like the temporary behavior file
            try:
                # do not clear imports for now, not working correctly (e.g., flexbe/flexbe_app#66)
                # only if specifically enabled
                if not self._switching and self._enable_clear_imports:
                    self._clear_imports()
                self._cleanup_behavior(beh_sel_msg.behavior_id)
            except (AttributeError, ImportError, OSError) as exc:
                Logger.localerr(f"Failed to clean up behavior '{be.name}': "
                                f'{be.beh_id}:\n  {str(exc)}')

            self._publish_stopped_status(be.beh_id)

            if not self._switching:
                Logger.localinfo(f"Behavior execution finished for '{be.name}': {be.beh_id}"
                                 f" with result '{str(result)}'")
                self._publish_ready_status()
                Logger.localinfo('\033[92m--- Behavior Engine finished - ready for more! ---\033[0m')

            Logger.localinfo(f'Behavior execution finished for id={be.beh_id}, exit thread!')
            self._running = False
            self._switching = False
            self._run_state_event.set()

    def _userdata_callback(self, request, response):
        userdata = []
        be = self.be
        state_machine = be._state_machine if be is not None else None
        if state_machine is not None:
            # get userdata from top-level behavior
            if state_machine._userdata:
                for key, data in state_machine._userdata._data.items():
                    # add userdata if it fits to the requested key (get all userdata for empty string)
                    if request.userdata_key in ('', key):
                        userdata.append(UserdataInfo(state=state_machine._name,
                                                     key=str(key),
                                                     type=type(data).__name__,
                                                     data=str(data)))
            # get userdata from sub-behaviors
            userdata = self._get_userdata_from_whole_sm(state_machine, userdata,
                                                        request.userdata_key,
                                                        str(state_machine._name) + '/')

            if len(userdata) > 0:
                # also print in terminal (better readability for complex message types)
                Logger.localinfo(
                    f'GetUserdata Service: Found {len(userdata)} '
                    f"occurrences of key='{request.userdata_key}' "
                    f"from be='{state_machine._name}'"
                )
                for ud in userdata:
                    Logger.localinfo(f"\tuser data key={ud.key}:\n{ud.data}\n{10 * '-'}")
                Logger.localinfo(f"{10 * '='} End get user data {10 * '='}")
                response.success = True
            else:
                response.success = False
            response.message = (f"Found {len(userdata)} occurrences of '{request.userdata_key}' "
                                f"from be='{state_machine._name}'")
            response.userdata = userdata
        else:
            response.success = False
            response.message = 'no state_machine running'
        return response

    # ==================================== #
    # Preparation of new behavior requests #
    # ------------------------------------ #

    def _report_execution_failure(self, exc, result):
        """Report behavior execution failures with mapped status and traceback."""
        import traceback
        try:
            status_code = map_exception_to_bestatus(exc, default=BEStatus.FAILED)
            self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                              behavior_id=self.be.beh_id,
                                              code=status_code))
            Logger.logerr(f'Behavior execution for {self.be.name}: {self.be.beh_id} failed (status={status_code})!\n%s'
                          % str(exc))
            Logger.localinfo(f"{traceback.format_exc().replace('%', '%%')}")  # Avoid single % in string
        except Exception as report_exc:  # pylint: disable=W0703
            print(f'Failed to report behavior execution failure: {report_exc}\n'
                  f'{traceback.format_exc()}', flush=True)
        return result or 'exception'  # only set result if not executed

    def _report_prepare_failure(self, beh_sel_msg, exc, message, clear_imports=False, cleanup_behavior=False):
        """Report preparation failures with mapped status code and optional import cleanup."""
        status_code = map_exception_to_bestatus(exc, default=BEStatus.ERROR)
        Logger.logerr(f'{message}:\n{exc}')
        self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                          behavior_id=beh_sel_msg.behavior_id,
                                          code=status_code))
        if clear_imports and self._enable_clear_imports:
            self._clear_imports()
        if cleanup_behavior:
            self._cleanup_behavior(beh_sel_msg.behavior_id)
        return None

    def _prepare_behavior(self, beh_sel_msg):
        # get sourcecode from ros package
        Logger.loginfo('--> Preparing new behavior...')
        try:
            behavior = self._behavior_lib.get_behavior(beh_sel_msg.behavior_key)
            if behavior is None:
                raise BehaviorLoadError(f'Behavior key not found: {beh_sel_msg.behavior_key}')
            be_filepath = self._behavior_lib.get_sourcecode_filepath(beh_sel_msg.behavior_key, add_tmp=True)
            if os.path.isfile(be_filepath):
                Logger.localwarn('Found a tmp version of the referred behavior! Assuming local test run.')
            else:
                be_filepath = self._behavior_lib.get_sourcecode_filepath(beh_sel_msg.behavior_key)

            with open(be_filepath, 'r') as be_file:
                be_content = be_file.read()

        except BehaviorLoadError as exc:
            return self._report_prepare_failure(beh_sel_msg, exc, 'Failed to retrieve behavior from library')
        except (AttributeError, KeyError, OSError, TypeError, ValueError) as exc:
            return self._report_prepare_failure(beh_sel_msg,
                                                BehaviorLoadError(f'Unexpected retrieval error: {exc}'),
                                                'Failed to retrieve behavior from library')

        # apply modifications if any
        try:
            file_content = ''
            last_index = 0
            for mod in beh_sel_msg.modifications:
                file_content += be_content[last_index:mod.index_begin] + mod.new_content
                last_index = mod.index_end
            file_content += be_content[last_index:]
            if zlib.adler32(file_content.encode()) & 0x7fffffff != beh_sel_msg.behavior_id:
                mismatch_msg = ('Checksum mismatch of behavior versions! \n'
                                'Attempted to load behavior: %s\n'
                                'Make sure that all computers are on the same version a.\n'
                                'Also try: ros2 run flexbe_widget clear_cache' % str(be_filepath))
                raise BehaviorLoadError(mismatch_msg)
            else:
                Logger.localinfo('Successfully applied %d modifications.' % len(beh_sel_msg.modifications))
        except BehaviorLoadError as exc:
            return self._report_prepare_failure(beh_sel_msg, exc, 'Failed to apply behavior modifications')
        except (AttributeError, TypeError, ValueError) as exc:
            return self._report_prepare_failure(beh_sel_msg,
                                                BehaviorLoadError(f'Unexpected behavior modification error: {exc}'),
                                                'Failed to apply behavior modifications')

        # create temp file for behavior class
        try:
            file_path = os.path.join(self._tmp_folder, f'tmp_{beh_sel_msg.behavior_id}.py')
            with open(file_path, 'w') as sc_file:
                sc_file.write(file_content)
        except OSError as exc:
            return self._report_prepare_failure(beh_sel_msg,
                                                BehaviorLoadError(f'Failed to create temporary file: {exc}'),
                                                'Failed to create temporary file for behavior class')

        # import temp class file and initialize behavior
        try:
            with self._track_imports():
                package = __import__('tmp_%d' % beh_sel_msg.behavior_id,
                                     fromlist=['tmp_%d' % beh_sel_msg.behavior_id])
                beclass = getattr(package, behavior['class'])
                if not isinstance(beclass, type) or beclass.__module__ != package.__name__:
                    raise BehaviorLoadError(
                        f"Behavior class '{behavior['class']}' is not defined in module '{package.__name__}'"
                    )
                be = beclass(self)
                Logger.localinfo(f"Created behavior '{be.name}' from package '{behavior['package']}'.")
        except (AttributeError, ImportError, TypeError, BehaviorLoadError) as exc:
            load_exc = BehaviorLoadError(f'Exception caught in behavior definition: {exc}')
            Logger.logerr(f'{load_exc}\nSee onboard terminal for more information.')
            import traceback
            Logger.localinfo(f"{traceback.format_exc().replace('%', '%%')}")  # Avoid single % in string
            return self._report_prepare_failure(beh_sel_msg,
                                                load_exc,
                                                'Exception caught in behavior definition',
                                                clear_imports=True,
                                                cleanup_behavior=True)

        if len(beh_sel_msg.arg_keys) != len(beh_sel_msg.arg_values):
            return self._report_prepare_failure(
                beh_sel_msg,
                BehaviorLoadError(f'Mismatched parameter arrays: {len(beh_sel_msg.arg_keys)} keys '
                                  f'!= {len(beh_sel_msg.arg_values)} values'),
                f"Invalid parameter payload for behavior key='{beh_sel_msg.behavior_key}'",
                cleanup_behavior=True
            )

        if len(beh_sel_msg.input_keys) != len(beh_sel_msg.input_values):
            return self._report_prepare_failure(
                beh_sel_msg,
                BehaviorLoadError(f'Mismatched input arrays: {len(beh_sel_msg.input_keys)} keys '
                                  f'!= {len(beh_sel_msg.input_values)} values'),
                f"Invalid input payload for behavior key='{beh_sel_msg.behavior_key}'",
                cleanup_behavior=True
            )

        # initialize behavior parameters
        if len(beh_sel_msg.arg_keys) > 0:
            Logger.localinfo('The following parameters will be used:')
        try:
            for i in range(len(beh_sel_msg.arg_keys)):
                # action call has empty string as default, not a valid param key
                if beh_sel_msg.arg_keys[i] == '':
                    continue
                found = be.set_parameter(beh_sel_msg.arg_keys[i], beh_sel_msg.arg_values[i])
                if found:
                    name_split = beh_sel_msg.arg_keys[i].rsplit('/', 1)
                    behavior = name_split[0] if len(name_split) == 2 else ''
                    key = name_split[-1]
                    suffix = ' (' + behavior + ')' if behavior != '' else ''
                    Logger.localinfo(key + ' = ' + beh_sel_msg.arg_values[i] + suffix)
                else:
                    Logger.localwarn(
                        f"Parameter '{beh_sel_msg.arg_keys[i]}' "
                        f"(set to '{beh_sel_msg.arg_values[i]}') not defined"
                    )
        except (AttributeError, IndexError, TypeError, ValueError) as exc:
            return self._report_prepare_failure(beh_sel_msg,
                                                BehaviorLoadError(f'Failed to initialize parameters: {exc}'),
                                                f"Failed to initialize parameters for behavior key='{beh_sel_msg.behavior_key}'",
                                                cleanup_behavior=True)

        # build state machine
        try:
            Logger.localinfo(f'Building state machine {beh_sel_msg.behavior_id} with '
                             f'autonomy level={beh_sel_msg.autonomy_level}.')
            be.set_up(beh_id=beh_sel_msg.behavior_id, autonomy_level=beh_sel_msg.autonomy_level, debug=False)
            be.prepare_for_execution(self._convert_input_data(beh_sel_msg.input_keys, beh_sel_msg.input_values))
            Logger.localinfo('State machine built.')
        except Exception as exc:  # pylint: disable=W0703
            # TODO: High-risk guardrail. Keep broad catch until behavior construction failure taxonomy is finalized.
            load_exc = BehaviorLoadError(f'Behavior construction failed: {exc}')
            Logger.logerr(f'{load_exc}\nSee onboard terminal for more information.')
            import traceback
            Logger.localinfo(f"{traceback.format_exc().replace('%', '%%')}")  # Avoid single % in string
            return self._report_prepare_failure(beh_sel_msg,
                                                load_exc,
                                                'Behavior construction failed',
                                                clear_imports=True,
                                                cleanup_behavior=True)

        Logger.localinfo(f'Finished behavior preparation for id={be.beh_id}!')
        return be

    # ================ #
    # Helper functions #
    # ---------------- #

    def _is_switchable(self, be):
        if self.be.name != be.name:
            Logger.logerr('Unable to switch behavior, names do not match:\ncurrent: %s <--> new: %s' %
                          (self.be.name, be.name))
            return False
        # locked inside
        # locked state exists in new behavior
        # ok, can switch
        return True

    def _cleanup_behavior(self, behavior_checksum):
        base_name = f'tmp_{behavior_checksum}'
        for filename in (f'{base_name}.py', f'{base_name}.pyc', f'{base_name}.pycc'):
            try:
                os.remove(os.path.join(self._tmp_folder, filename))
            except OSError:
                pass

        pycache_dir = os.path.join(self._tmp_folder, '__pycache__')
        if os.path.isdir(pycache_dir):
            for filename in os.listdir(pycache_dir):
                if filename.startswith(base_name) and filename.endswith('.pyc'):
                    try:
                        os.remove(os.path.join(pycache_dir, filename))
                    except OSError:
                        pass

    def _clear_imports(self):
        for module in self._tracked_imports:
            if module in sys.modules:
                del sys.modules[module]
        self._tracked_imports = []

    def _cleanup_tempdir(self):
        try:
            if self._tmp_folder in sys.path:
                sys.path.remove(self._tmp_folder)
        except ValueError:
            pass
        shutil.rmtree(self._tmp_folder, ignore_errors=True)

    def _convert_input_data(self, keys, values):
        result = {}
        for k, v in zip(keys, values):
            # action call has empty string as default, not a valid input key
            if k == '':
                continue
            try:
                result[k] = self._convert_dict(cast(v))
            except ValueError:
                # unquoted strings will raise a ValueError, so leave it as string in this case
                result[k] = str(v)
            except SyntaxError as se:
                Logger.loginfo(f"Unable to parse input value for key '{k}', assuming string:\n{str(v)}\n{str(se)}")
                result[k] = str(v)
        return result

    def _heartbeat_worker(self):

        # Periodically update our local logger permissions
        Logger.check_local_enabled()

        # Publish a heartbeat while node is alive
        be = self.be
        if be is not None:
            heartbeat = be.get_latest_status()
        else:
            heartbeat = self._idle_heartbeat

        # Logger.localinfo(f'Heartbeat: {heartbeat.behavior_id}: {heartbeat.current_state_checksum } '
        #                  f'- running {self._running} switching {self._switching} ')
        self._heartbeat_pub.publish(heartbeat)

        if not self._running and not self._switching:
            # If the statemachine is not active
            if self._trigger_ready:
                # re-pub READY message periodically in case of FlexBE OCS reset
                self._publish_ready_status()
                self._trigger_ready = False
                self._ready_counter = 0
            else:
                self._ready_counter += 1
                if self._ready_counter > 9:
                    self._trigger_ready = True
        else:
            self._trigger_ready = False
            self._ready_counter = 0

    def _convert_dict(self, o):
        if isinstance(o, list):
            return [self._convert_dict(e) for e in o]

        if isinstance(o, dict):
            return self._attr_dict((k, self._convert_dict(v)) for k, v in list(o.items()))

        return o

    def _publish_ready_status(self):
        """Publish the cached READY status message with a refreshed timestamp."""
        try:
            self._ready_status.stamp = self.get_clock().now().to_msg()
            self._status_pub.publish(self._ready_status)  # Publish regardless of subscribers for latched
        except (InvalidHandle, RuntimeError):
            pass  # Publisher destroyed during teardown.

    @staticmethod
    def _get_switch_leaf_states(active_states):
        """Return the active non-container states from a deep-state path tuple."""
        if active_states is None:
            return ()
        return tuple(state for state in active_states if state is not None and not isinstance(state, StateMachine))

    def _publish_stopped_status(self, behavior_id):
        """Publish STOPPED after the last outcome for a run and before READY or restart."""
        try:
            self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(),
                                              behavior_id=behavior_id,
                                              code=BEStatus.STOPPED))
        except (InvalidHandle, RuntimeError):
            # Teardown can destroy publishers while a behavior thread is unwinding.
            return

    def _get_userdata_from_whole_sm(self, state_machine, userdata, userdata_key, path):
        # iterate recursively through all sub-behaviors
        for subbehavior in state_machine._states:
            # check if userdata available
            if isinstance(subbehavior, StateMachine):
                if subbehavior._userdata:
                    for key, data in subbehavior._userdata._data.items():
                        # add userdata if fits to the requested key (get all userdata for empty string)
                        if userdata_key in ('', key):
                            userdata.append(UserdataInfo(state=path + subbehavior.name + '/',
                                                         key=str(key),
                                                         type=type(data).__name__,
                                                         data=str(data)))
                self._get_userdata_from_whole_sm(subbehavior, userdata, userdata_key, path + subbehavior.name + '/')
        return userdata

    class _attr_dict(dict):
        __getattr__ = dict.__getitem__

    @contextlib.contextmanager
    def _track_imports(self):
        previous_modules = set(sys.modules.keys())
        try:
            yield
        finally:
            self._tracked_imports.extend(set(sys.modules.keys()) - previous_modules)
