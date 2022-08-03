#!/usr/bin/env python
import rclpy
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
import os
import sys
import inspect
import tempfile
import threading
import time
import zlib
import contextlib
from ast import literal_eval as cast

from flexbe_core import BehaviorLibrary, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_msgs.msg import BehaviorSelection, BehaviorSync, BEStatus, CommandFeedback
from std_msgs.msg import Empty


class FlexbeOnboard(Node):
    """
    Controls the execution of robot behaviors.
    """

    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('flexbe_onboard')

        ProxyPublisher._initialize(self)
        ProxySubscriberCached._initialize(self)
        Logger.initialize(self)

        self.be = None
        self._tracked_imports = list()
        # prepare temp folder
        self._tmp_folder = tempfile.mkdtemp()
        sys.path.append(self._tmp_folder)

        # prepare manifest folder access
        self._behavior_lib = BehaviorLibrary(self)

        # prepare communication
        self.status_topic = 'flexbe/status'
        self.feedback_topic = 'flexbe/command_feedback'
        self._pub = ProxyPublisher({self.feedback_topic: CommandFeedback, 'flexbe/heartbeat': BehaviorSync})
        self._pub.createPublisher(self.status_topic, BEStatus)

        # listen for new behavior to start
        try:
            self._enable_clear_imports = self.get_parameter('enable_clear_imports').get_parameter_value()
        except ParameterNotDeclaredException as e:
            self.declare_parameter('enable_clear_imports', False)
            self._enable_clear_imports = self.get_parameter('enable_clear_imports').get_parameter_value()

        self._trigger_ready = True
        self._running = False
        self._run_lock = threading.Lock()
        self._switching = False
        self._switch_lock = threading.Lock()
        self._behavior_id = -1
        self._current_state_checksum = -1
        self._sub = ProxySubscriberCached()
        self._sub.subscribe('flexbe/start_behavior', BehaviorSelection, self._behavior_callback, id=id(self))

        time.sleep(0.5)  # wait for publishers etc to really be set up
        self._execute_heartbeat() # Will also publish BEStatus ready until first behavior received
        Logger.loginfo('\033[92m--- Behavior Engine ready for first behavior! ---\033[0m')

    def _behavior_callback(self, msg):
        self._trigger_ready = False # We have received the first behavior request
        thread = threading.Thread(target=self._behavior_execution, args=[msg])
        thread.daemon = True
        thread.start()

    # =================== #
    # Main execution loop #
    # ------------------- #

    def _behavior_execution(self, msg):
        # sending a behavior while one is already running is considered as switching
        if not rclpy.ok():
            self._cleanup_tempdir()

        if self._running:
            Logger.loginfo('--> Initiating behavior switch...')
            self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['received']))

        # construct the behavior that should be executed
        be = self._prepare_behavior(msg)
        if be is None:
            Logger.logerr('Dropped behavior start request because preparation failed.')
            if self._running:
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['failed']))
            else:
                self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), code=BEStatus.READY))
                Logger.loginfo('\033[92m--- Behavior Engine ready to try again! ---\033[0m')
            return

        # perform the behavior switch if required
        Logger.localinfo("Behavior Engine - get switch lock ...")
        with self._switch_lock:
            if self._running:
                assert self.be is not None, "Must have an active behavior here!"
                self._switching = True
                Logger.localinfo(f"Behavior Engine - prepare to switch running behavior {self.be.name}: {self.be.id}...")
                self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['start']))

                # ensure that switching is possible
                if not self._is_switchable(be):
                    Logger.logerr('Dropped behavior start request because switching is not possible.')
                    self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['not_switchable']))
                    return

                self._pub.publish(self.status_topic,
                                  BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=self.be.id, code=BEStatus.SWITCHING))
                # wait if running behavior is currently starting or stopping
                rate = self.create_rate(100, self.get_clock())
                active_state = None
                while rclpy.ok():
                    active_state = self.be.get_current_state()
                    if active_state is not None or not self._running:
                        break
                    rate.sleep()

                # extract the active state if any
                if active_state is not None:
                    Logger.localinfo(f'Behavior Engine - {self.be.name}: {self.be.id} switching behaviors from active state {active_state.name} ...')
                    try:
                        be.prepare_for_switch(active_state)
                        self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['prepared']))
                    except Exception as e:
                        Logger.logerr('Failed to prepare behavior switch:\n%s' % str(e))
                        self._pub.publish(self.feedback_topic, CommandFeedback(command="switch", args=['failed']))
                        # Let us know that old behavior is still running
                        self._pub.publish(self.status_topic,
                                          BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=self.be.id, code=BEStatus.RUNNING))
                        return
                    # stop the rest
                    Logger.localinfo(f'Behavior Engine - {self.be.name}: {self.be.id} - preempt active state  {active_state.name} ...')
                    self.be.preempt()
                else:
                    Logger.localinfo(f'Behavior Engine - {self.be.name}: {self.be.id} no active state to preempt (but Running={self._running}?)!')

        # execute the behavior
        Logger.localinfo(f'Waiting on prior behavior to shutdown ...')
        with self._run_lock:
            assert self.be is None, "Run lock with old behavior active?"
            self._running = True
            self.be = be

            result = None
            try:
                Logger.localinfo(f'Behavior Engine - behavior {self.be.name}: {self.be.id} ready, begin startup ...')
                Logger.loginfo('BE Starting [%s : %s]' % (be.name, msg.behavior_checksum))
                self.be.confirm()
                Logger.localinfo(f'Behavior Engine - behavior {self.be.name}: {self.be.id} confirmation.')
                args = [self.be.requested_state_path] if self.be.requested_state_path is not None else []
                Logger.localinfo(f'Behavior Engine - behavior {self.be.name}: {self.be.id} BEStatus STARTED.')
                self._pub.publish(self.status_topic,
                                  BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=self.be.id, code=BEStatus.STARTED, args=args))

                # Do the behavior
                Logger.localinfo(f'Behavior Engine - behavior {self.be.name}: {self.be.id} begin execution ...')
                result = self.be.execute()
                Logger.localinfo(f'Behavior Engine - {self.be.name}: {self.be.id} done execute with result={result}')
                self._pub.publish(self.status_topic,
                                  BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=self.be.id, code=BEStatus.FINISHED, args=[str(result)]))
            except Exception as e:
                self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=self.be.id, code=BEStatus.FAILED))
                Logger.logerr(f'Behavior execution for {self.be.name}: {self.be.id} failed!\n%s' % str(e))
                import traceback
                Logger.loginfo(f'''{traceback.format_exc().replace("%", "%%")}''') # Avoid single % in string
                result = result or "exception"  # only set result if not executed

            # done, remove left-overs like the temporary behavior file
            try:
                # do not clear imports for now, not working correctly (e.g., flexbe/flexbe_app#66)
                # only if specifically enabled
                if not self._switching and self._enable_clear_imports:
                    self._clear_imports()
                self._cleanup_behavior(msg.behavior_checksum)
            except Exception as e:
                self.get_logger().error(f'Failed to clean up behavior {self.be.name}: {self.be.id}:\n%s' % str(e))

            if not self._switching:
                Logger.localinfo(f'Behavior execution finished for {self.be.name}: {self.be.id} with result {str(result)}')
                self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), code=BEStatus.READY))
                Logger.loginfo('\033[92m--- Behavior Engine finished - ready for more! ---\033[0m')

            self._running = False
            self._switching = False
            self.be = None

    # ==================================== #
    # Preparation of new behavior requests #
    # ------------------------------------ #

    def _prepare_behavior(self, msg):
        # get sourcecode from ros package
        Logger.loginfo('--> Preparing new behavior...')
        try:
            behavior = self._behavior_lib.get_behavior(msg.behavior_id)
            if behavior is None:
                raise ValueError(msg.behavior_id)
            be_filepath = self._behavior_lib.get_sourcecode_filepath(msg.behavior_id, add_tmp=True)
            if os.path.isfile(be_filepath):
                be_file = open(be_filepath, "r")
                self.get_logger().warn("Found a tmp version of the referred behavior! Assuming local test run.")
            else:
                be_filepath = self._behavior_lib.get_sourcecode_filepath(msg.behavior_id)
                be_file = open(be_filepath, "r")
            try:
                be_content = be_file.read()
            finally:
                be_file.close()
        except Exception as e:
            Logger.logerr('Failed to retrieve behavior from library:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # apply modifications if any
        try:
            file_content = ""
            last_index = 0
            for mod in msg.modifications:
                file_content += be_content[last_index:mod.index_begin] + mod.new_content
                last_index = mod.index_end
            file_content += be_content[last_index:]
            if zlib.adler32(file_content.encode()) & 0x7fffffff != msg.behavior_checksum:
                mismatch_msg = ("Checksum mismatch of behavior versions! \n"
                                "Attempted to load behavior: %s\n"
                                "Make sure that all computers are on the same version a.\n"
                                "Also try: ros2 run flexbe_widget clear_cache" % str(be_filepath))
                raise Exception(mismatch_msg)
            else:
                self.get_logger().info("Successfully applied %d modifications." % len(msg.modifications))
        except Exception as e:
            Logger.logerr('Failed to apply behavior modifications:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # create temp file for behavior class
        try:
            file_path = os.path.join(self._tmp_folder, 'tmp_%d.py' % msg.behavior_checksum)
            with open(file_path, "w") as sc_file:
                sc_file.write(file_content)
        except Exception as e:
            Logger.logerr('Failed to create temporary file for behavior class:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # import temp class file and initialize behavior
        try:
            with self._track_imports():
                package = __import__("tmp_%d" % msg.behavior_checksum, fromlist=["tmp_%d" % msg.behavior_checksum])
                clsmembers = inspect.getmembers(package, lambda member: (inspect.isclass(member) and
                                                                         member.__module__ == package.__name__))
                beclass = clsmembers[0][1]
                be = beclass(self)
            self.get_logger().info('Behavior ' + be.name + ' created.')
        except Exception as e:
            Logger.logerr('Exception caught in behavior definition:\n%s\n'
                          'See onboard terminal for more information.' % str(e))
            import traceback
            Logger.localinfo(f'''{traceback.format_exc().replace("%", "%%")}''') # Avoid single % in string
            self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            if self._enable_clear_imports:
                self._clear_imports()
            return

        # initialize behavior parameters
        if len(msg.arg_keys) > 0:
            self.get_logger().info('The following parameters will be used:')
        try:
            for i in range(len(msg.arg_keys)):
                # action call has empty string as default, not a valid param key
                if msg.arg_keys[i] == '':
                    continue
                found = be.set_parameter(msg.arg_keys[i], msg.arg_values[i])
                if found:
                    name_split = msg.arg_keys[i].rsplit('/', 1)
                    behavior = name_split[0] if len(name_split) == 2 else ''
                    key = name_split[-1]
                    suffix = ' (' + behavior + ')' if behavior != '' else ''
                    self.get_logger().info(key + ' = ' + msg.arg_values[i] + suffix)
                else:
                    self.get_logger().warn('Parameter ' + msg.arg_keys[i] + ' (set to ' + msg.arg_values[i] + ') not defined')
        except Exception as e:
            Logger.logerr('Failed to initialize parameters:\n%s' % str(e))
            self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            return

        # build state machine
        try:
            be.set_up(id=msg.behavior_checksum, autonomy_level=msg.autonomy_level, debug=False)
            be.prepare_for_execution(self._convert_input_data(msg.input_keys, msg.input_values))
            self.get_logger().info('State machine built.')
        except Exception as e:
            Logger.logerr('Behavior construction failed!\n%s\n'
                          'See onboard terminal for more information.' % str(e))
            import traceback
            Logger.localinfo(f'''{traceback.format_exc().replace("%", "%%")}''') # Avoid single % in string
            self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), behavior_id=msg.behavior_checksum, code=BEStatus.ERROR))
            if self._enable_clear_imports:
                self._clear_imports()
            return

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
        file_path = os.path.join(self._tmp_folder, 'tmp_%d.pyc' % behavior_checksum)
        try:
            os.remove(file_path)
        except OSError:
            pass
        try:
            os.remove(file_path + 'c')
        except OSError:
            pass

    def _clear_imports(self):
        for module in self._tracked_imports:
            if module in sys.modules:
                del sys.modules[module]
        self._tracked_imports = list()

    def _cleanup_tempdir(self):
        try:
            os.remove(self._tmp_folder)
        except OSError:
            pass

    def _convert_input_data(self, keys, values):
        result = dict()
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
                Logger.loginfo('Unable to parse input value for key "%s", assuming string:\n%s\n%s' %
                               (k, str(v), str(se)))
                result[k] = str(v)
        return result

    def _execute_heartbeat(self):
        thread = threading.Thread(target=self._heartbeat_worker)
        thread.daemon = True
        thread.start()

    def _heartbeat_worker(self):
        ready_counter = 0

        while True:

            # Publish a heartbeat while node is alive
            be = self.be
            if be is not None:
                heartbeat = be.get_latest_status()
            else:
                heartbeat = BehaviorSync()

            self._pub.publish('flexbe/heartbeat', heartbeat)

            #Logger.localinfo(f'Heartbeat: {heartbeat.behavior_id}: {heartbeat.current_state_checksum } - running {self._running} switching {self._switching} ')
            if not self._running and not self._switching:
                # If the statemachine is not active
                if self._trigger_ready:
                    # re-pub READY message periodically in case of FlexBE OCS reset
                    self._pub.publish(self.status_topic, BEStatus(stamp=self.get_clock().now().to_msg(), code=BEStatus.READY))
                    self._trigger_ready = False
                    ready_counter = 0
                else:
                    ready_counter += 1
                    if ready_counter > 9:
                        self._trigger_ready = True
            else:
                self._trigger_ready = False
                ready_counter = 0

            time.sleep(1)

    def _convert_dict(self, o):
        if isinstance(o, list):
            return [self._convert_dict(e) for e in o]
        elif isinstance(o, dict):
            return self._attr_dict((k, self._convert_dict(v)) for k, v in list(o.items()))
        else:
            return o

    class _attr_dict(dict):
        __getattr__ = dict.__getitem__

    @contextlib.contextmanager
    def _track_imports(self):
        previous_modules = set(sys.modules.keys())
        try:
            yield
        finally:
            self._tracked_imports.extend(set(sys.modules.keys()) - previous_modules)
