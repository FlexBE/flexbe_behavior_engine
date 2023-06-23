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


"""
Default context for a test case.

Use as a 'with' statement and run 'verify' to check whether the context is valid.
"""

import os
import re
import rclpy
import time

from rclpy.exceptions import ParameterNotDeclaredException
from ament_index_python.packages import get_package_share_directory

from .logger import Logger


# class Callback(roslaunch.pmon.ProcessListener):
#     def __init__(self, callback, node):
#         self._callback = callback
#         self._node = node
#
#     def process_died(self, process_name, exit_code):
#         self._node.get_logger().info("Process {} exited with {}".format(process_name, exit_code))
#         self._callback(process_name, exit_code)


class TestContext:
    """
    Default context for a test case.

    Use as a 'with' statement and run 'verify' to check whether the context is valid.
    """

    __test__ = False  # Do not pytest this class (it is the test!)

    def __init__(self):
        pass

    def __enter__(self):
        pass

    def ok(self):
        """Return ok status (default loop check)."""
        return rclpy.ok()

    def verify(self):
        return True

    def spin_once(self):
        pass

    def __exit__(self, exception_type, exception_value, traceback):
        pass

    def wait_for_finishing(self):
        pass

    @property
    def success(self):
        return True


class PyTestContext(TestContext):
    """Pylint based state tests uses counter and/or timeout_sec to control execute loop."""

    def __init__(self, timeout_sec=None, max_cnt=50):
        super().__init__()
        self._cnt = 0
        self._max_cnt = None
        self._time_out = None
        if max_cnt is not None:
            self._max_cnt = int(max_cnt)  # Allow string or float conversion

        if timeout_sec is not None:
            self._time_out = time.time() + float(timeout_sec)

        assert self._max_cnt is not None or self._time_out is not None, "Must have either timeout or max cnt set!"

    def ok(self):
        """Return ok status based on time and count for pytests."""
        if self._time_out is not None and time.time() > self._time_out:
            return False

        self._cnt += 1
        if self._max_cnt is not None and self._cnt > self._max_cnt:
            return False

        return True


class LaunchContext(TestContext):
    """Test context that runs a specified launch file configuration."""

    def __init__(self, node, launch_config, wait_cond="True"):
        self._node = node
        Logger.initialize(node)

        try:
            self._run_id = self._node.get_parameter('/run_id').get_parameter_value()
        except ParameterNotDeclaredException:  # pylint: disable=W0703
            self._node.get_logger().error('Unable to get parameter: /run_id')

        launchpath = None
        launchcontent = None

        self._launched_proc_names = []
        self._exit_codes = {}

        # load from system path
        if launch_config.startswith('~') or launch_config.startswith('/'):
            launchpath = os.path.expanduser(launch_config)
        # load from package path
        elif re.match(r'.+\.launch$', launch_config):
            # rp = rospkg.RosPack()
            # pkgpath = rp.get_path(launch_config.split('/')[0])

            pkgpath = get_package_share_directory(launch_config.split('/')[0])
            launchpath = os.path.join(pkgpath, '/'.join(launch_config.split('/')[1:]))
        # load from config definition
        else:
            launchcontent = launch_config

        # launchconfig = roslaunch.config.ROSLaunchConfig()
        # loader = roslaunch.xmlloader.XmlLoader()
        # if launchpath is not None:
        #     loader.load(launchpath, launchconfig, verbose=False)
        # else:
        #     loader.load_string(launchcontent, launchconfig, verbose=False)
        # self._launchrunner = roslaunch.launch.ROSLaunchRunner(self._run_id, launchconfig)
        self._launchrunner = None
        self._wait_cond = None
        self._valid = False
        raise NotImplementedError(f"Not implemented for ROS 2 - TODO!\n {launchpath}\n {launchcontent}\n{10*'='}")
        # def store(process_name, exit_code):
        #     self._exit_codes[process_name] = exit_code
        # self._launchrunner.add_process_listener(Callback(store))
        # self._wait_cond = wait_cond
        # self._valid = True

    def __enter__(self):
        raise NotImplementedError("Not implemented for ROS 2 - TODO!")
        self._launchrunner.launch()
        self._launchrunner.spin_once()
        Logger.print_positive('launchfile running')
        self._valid = True

        self._launched_proc_names = [p.name for p in self._launchrunner.pm.procs]

        try:
            check_running_rate = self._node.create_rate(10, self._node.get_clock())
            is_running = False
            while not is_running:
                is_running = eval(self._wait_cond)
                check_running_rate.sleep()
            Logger.print_positive('waiting condition satisfied')
            self._node.destroy_rate(check_running_rate)
        except Exception as e:
            self._valid = False
            Logger.print_negative('unable to check waiting condition:\n\t%s' % str(e))

    def verify(self):
        return self._valid

    def spin_once(self):
        self._launchrunner.spin_once()

    def wait_for_finishing(self):
        check_exited_rate = self._node.create_rate(10, self._node.get_clock())
        self._node.get_logger().info("Waiting for all launched nodes to exit")
        while not all(name in self._exit_codes for name in self._launched_proc_names):
            check_exited_rate.sleep()
        self._node.destroy_rate(check_exited_rate)

    def __exit__(self, exception_type, exception_value, traceback):
        self._launchrunner.stop()
        Logger.print_positive('launchfile stopped')

    @property
    def success(self):
        return not any(code > 0 for code in self._exit_codes.values())
