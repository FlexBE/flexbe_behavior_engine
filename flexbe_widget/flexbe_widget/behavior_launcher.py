#!/usr/bin/env python

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


"""Node to launch FlexBE behaviors."""

from datetime import datetime
import difflib
import getopt
import os
import sys
import threading
import yaml
import zlib

import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_interface_path

from std_msgs.msg import Int32, String

from flexbe_msgs.msg import BehaviorModification, BehaviorRequest, BehaviorSelection, BEStatus, ContainerStructure

from flexbe_core import BehaviorLibrary, Logger, MIN_UI_VERSION
from flexbe_core.core import StateMap
from flexbe_core.core.topics import Topics


class BehaviorLauncher(Node):
    """Node to launch FlexBE behaviors."""

    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('flexbe_widget')

        self._ready_event = threading.Event()

        self._sub = self.create_subscription(BehaviorRequest, Topics._REQUEST_BEHAVIOR_TOPIC, self._request_callback, 100)
        self._version_sub = self.create_subscription(String, Topics._UI_VERSION_TOPIC, self._version_callback, 1)
        self._status_sub = self.create_subscription(BEStatus, Topics._ONBOARD_STATUS_TOPIC, self._status_callback, 100)

        self._pub = self.create_publisher(BehaviorSelection, Topics._START_BEHAVIOR_TOPIC, 100)
        self._status_pub = self.create_publisher(BEStatus, Topics._ONBOARD_STATUS_TOPIC, 100)
        self._mirror_pub = self.create_publisher(ContainerStructure, Topics._MIRROR_STRUCTURE_TOPIC, 100)
        self._heartbeat_pub = self.create_publisher(Int32, Topics._LAUNCHER_HEARTBEAT_TOPIC, 2)

        self._behavior_lib = BehaviorLibrary(self)

        # Require periodic events in case behavior is not connected to allow orderly shutdown
        self._heartbeat_timer = self.create_timer(2.0, self.heartbeat_timer_callback)

        self.get_logger().info("%d behaviors available, ready for start request." % self._behavior_lib.count_behaviors())

    def heartbeat_timer_callback(self):
        """
        Allow monitoring of Launcher liveness.

        Guarantee some event triggers wake up so that we can catch Ctrl-C in case where no active messages are available.
        """
        self._heartbeat_pub.publish(Int32(data=self.get_clock().now().seconds_nanoseconds()[0]))

    def _status_callback(self, msg):
        if msg.code in [BEStatus.READY, BEStatus.FINISHED, BEStatus.FAILED, BEStatus.ERROR, BEStatus.RUNNING, BEStatus.STARTED]:
            self.get_logger().info(f"BE status code={msg.code} received - READY for new behavior!")
            self._ready_event.set()
        else:
            self.get_logger().info(f"BE status code={msg.code} received ")

    def _request_callback(self, msg):
        """Process request in separate thread to avoid blocking callbacks."""
        if not self._ready_event.is_set():
            Logger.logerr("Behavior engine is not ready - cannot process start request!")
        else:
            # self._ready_event.clear()  # require a new ready signal after publishing
            # thread = threading.Thread(target=self._process_request, args=[msg])
            # thread.daemon = True
            # thread.start()
            # Not waiting in process request, so safe to not block callback
            self._process_request(msg)

    def _process_request(self, msg):
        self.get_logger().info(f"Got message from request behavior for {msg.behavior_name}")
        be_key, behavior = self._behavior_lib.find_behavior(msg.behavior_name)
        if be_key is None:
            self.get_logger().error("Did not find behavior with requested name: %s" % msg.behavior_name)
            self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(), code=BEStatus.ERROR))
            return

        self.get_logger().info(f"""Processing request using behavior '{behavior["name"]}' """
                               f"""from package='{behavior["package"]}' with key={be_key} ...""")

        be_selection = BehaviorSelection()
        be_selection.behavior_key = be_key
        be_selection.autonomy_level = msg.autonomy_level
        try:
            for k, v in zip(msg.arg_keys, msg.arg_values):
                if k.startswith('/YAML:'):
                    key = k.replace('/YAML:', '/', 1)
                    path = v.split(':')[0]
                    ns = v.split(':')[1]
                    if path.startswith('~') or path.startswith('/'):
                        yamlpath = os.path.expanduser(path)
                    else:
                        yamlpath = os.path.join(get_interface_path(path.split('/')[0]), '/'.join(path.split('/')[1:]))
                    with open(yamlpath, 'r') as f:
                        content = getattr(yaml, 'unsafe_load', yaml.load)(f)
                    if ns != '' and ns in content:
                        content = content[ns]
                    be_selection.arg_keys.append(key)
                    be_selection.arg_values.append(yaml.dump(content))
                else:
                    be_selection.arg_keys.append(k)
                    be_selection.arg_values.append(v)
        except Exception as exc:
            self.get_logger().warn("Failed to parse and substitute behavior arguments, "
                                   f"will use direct input.\n {type(exc)} - {str(exc)}")
            be_selection.arg_keys = msg.arg_keys
            be_selection.arg_values = msg.arg_values

        container_map = StateMap()
        be_structure = ContainerStructure()
        be_structure.containers = msg.structure
        for container in be_structure.containers:
            # self.get_logger().info(f"BELauncher: request_callback: adding container {container.path} ...")
            container_map.add_state(container.path, container)
        # self.get_logger().info(f"BELauncher: request_callback: {msg.structure}")

        try:
            be_filepath_new = self._behavior_lib.get_sourcecode_filepath(be_key)
        except Exception:  # pylint: disable=W0703
            self.get_logger().error("Could not find behavior package '%s'" % (behavior["package"]))
            self.get_logger().info("Have you built and updated your setup after creating the behavior?")
            self._status_pub.publish(BEStatus(stamp=self.get_clock().now().to_msg(), code=BEStatus.ERROR))
            return

        with open(be_filepath_new, "r") as f:
            be_content_new = f.read()

        self.get_logger().info("Check for behavior change ...")
        be_filepath_old = self._behavior_lib.get_sourcecode_filepath(be_key, add_tmp=True)
        if not os.path.isfile(be_filepath_old):
            be_selection.behavior_id = zlib.adler32(be_content_new.encode()) & 0x7fffffff
            if msg.autonomy_level != 255:
                be_structure.behavior_id = be_selection.behavior_id
                # self.get_logger().info(f"BELauncher: request_callback publish structure : {be_structure}")
                self._mirror_pub.publish(be_structure)
            self._ready_event.clear()  # require a new ready signal after publishing
            self._pub.publish(be_selection)
            self.get_logger().info("No changes to behavior version - restart")
            return

        with open(be_filepath_old, "r") as f:
            be_content_old = f.read()

        sqm = difflib.SequenceMatcher(a=be_content_old, b=be_content_new)
        diffs = [x[1] for x in sqm.get_grouped_opcodes(0)]
        for opcode, a0, a1, b0, b1 in diffs:  # pylint: disable=W0612
            content = be_content_new[b0:b1]
            be_selection.modifications.append(BehaviorModification(index_begin=a0,
                                                                   index_end=a1,
                                                                   new_content=content))

        be_selection.behavior_id = zlib.adler32(be_content_new.encode()) & 0x7fffffff
        if msg.autonomy_level != 255:
            be_structure.behavior_id = be_selection.behavior_id
            self._mirror_pub.publish(be_structure)

        self._ready_event.clear()  # Force a new ready message before processing
        self._pub.publish(be_selection)
        self.get_logger().info(f"New behavior key={be_selection.behavior_key} published "
                               f"with checksum id = {be_selection.behavior_id}- start!")

    def _version_callback(self, msg):
        vui = BehaviorLauncher._parse_version(msg.data)
        vex = BehaviorLauncher._parse_version(MIN_UI_VERSION)
        if vui < vex:
            Logger.logwarn('FlexBE App needs to be updated!\n'
                           f'Behavior launcher requires at least version {MIN_UI_VERSION}, '
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


def usage():
    print("Usage: Soon...")


def behavior_launcher_main(node_args=None):
    opts = None
    args = None

    try:
        print(sys.argv)
        stop_index = len(sys.argv)
        try:
            # Stop processing after --ros-args
            stop_index = next(i for i in range(len(sys.argv)) if sys.argv[i] == "--ros-args")
        except Exception:  # pylint: disable=W0703
            pass
        opts, args = getopt.getopt(sys.argv[1:stop_index], "hb:a:", ["help", "behavior=", "autonomy="])
    except getopt.GetoptError as exc:
        usage()
        print(exc)
        print("Continue after exception ")

    behavior = ""
    autonomy = 255

    if opts:
        for opt, arg in opts:
            if opt in ("-h", "--help"):
                usage()
                sys.exit()
            elif opt in ("-b", "--behavior"):
                behavior = arg
            elif opt in ("-a", "--autonomy"):
                autonomy = int(arg)
    ignore_args = ['__node', '__log']  # auto-set by roslaunch

    print("Set up behavior_launcher ROS connections ...", flush=True)
    node_args = sys.argv[stop_index:]
    rclpy.init(args=node_args,
               signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)  # We will handle shutdown

    launcher = BehaviorLauncher()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(launcher)

    if behavior != "":
        print(f"Set up behavior_launcher with '{behavior}' ...", flush=True)
        for _ in range(100):
            # Let stuff get going before launching behavior request
            executor.spin_once(timeout_sec=0.001)

        request = BehaviorRequest(behavior_name=behavior, autonomy_level=autonomy)
        for arg in args:
            if ':=' not in arg:
                continue
            key, val = arg.split(':=', 1)
            if key in ignore_args:
                continue
            request.arg_keys.append('/' + key)
            request.arg_values.append(val)
        print(f"Add callback request for '{behavior}' ...", flush=True)
        executor.spin_once(timeout_sec=0.001)

        # Set up a callable as a future task so we don't block before starting to spin
        def initial_request():
            return launcher._request_callback(request)
        future = executor.create_task(initial_request)
    else:
        future = None

    try:
        # Wait for ctrl-c to stop the application
        if future:
            print("Run initial request of behavior_launcher spinner ...", flush=True)
            executor.spin_until_future_complete(future, timeout_sec=10.0)
            if future.done():
                Logger.info("Initial behavior launcher request is sent!")
            else:
                Logger.error("Initial request failed to send - timed out before complete!")

        print("Start behavior_launcher spinner ...", flush=True)
        executor.spin()
    except KeyboardInterrupt:
        print(f"Keyboard interrupt request  at {datetime.now()} - ! Shut the behavior launcher down!", flush=True)
    except Exception as exc:
        print(f"Exception in executor       at {datetime.now()} - ! {type(exc)}\n  {exc}", flush=True)
        import traceback
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    try:
        launcher.destroy_node()
    except Exception as exc:  # pylint: disable=W0703
        print(f"Exception from destroy behavior launcher node at {datetime.now()}: {type(exc)}\n{exc}", flush=True)
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)

    print(f"Done with behavior launcher at {datetime.now()}!", flush=True)
    try:
        rclpy.try_shutdown()
    except Exception as exc:  # pylint: disable=W0703
        print(f"Exception from rclpy.try_shutdown for behavior launcher: {type(exc)}\n{exc}", flush=True)
        print(f"{traceback.format_exc().replace('%', '%%')}", flush=True)


if __name__ == '__main__':
    behavior_launcher_main()
