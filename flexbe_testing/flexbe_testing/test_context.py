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
import shutil
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

import rclpy

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

    def __init__(self, node=None, execute_wait=0.01):
        """Initialize."""
        self._node = node
        self._execute_wait = execute_wait

    def __enter__(self):
        """Enter test."""
        pass

    def ok(self):
        """Return ok status (default loop check)."""
        if self._node is None:
            return rclpy.ok()
        return rclpy.ok(context=self._node.context)

    def verify(self):
        """Verify test results."""
        return True

    def spin_once(self):
        """Spin event loop once."""
        pass

    def __exit__(self, exception_type, exception_value, traceback):
        """Exit the test."""
        pass

    def wait_for_finishing(self):
        """Wait for test to finish."""
        pass

    def sleep(self):
        """Sleep for the configured wait duration."""
        if self._execute_wait is not None:
            time.sleep(self._execute_wait)

    @property
    def success(self):
        """Check test success."""
        return True


class PyTestContext(TestContext):
    """Pylint based state tests uses counter and/or timeout_sec to control execute loop."""

    def __init__(self, node=None, timeout_sec=None, max_cnt=50, execute_wait=0.01):
        """Initialize the PyTestContext."""
        super().__init__(node, execute_wait)
        self._cnt = 0
        self._max_cnt = None
        self._time_out = None
        if max_cnt is not None:
            self._max_cnt = int(max_cnt)  # Allow string or float conversion

        if timeout_sec is not None:
            self._time_out = time.time() + float(timeout_sec)

        assert self._max_cnt is not None or self._time_out is not None, 'Must have either timeout or max cnt set!'

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

    class _EvalRclpyProxy:
        """Expose the live rclpy module while allowing wait_for_message to be overridden."""

        def __init__(self, module, wait_for_message=None):
            self._module = module
            self.wait_for_message = wait_for_message

        def __getattr__(self, name):
            return getattr(self._module, name)

    _RUNNER_CODE = """
import sys
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

service = LaunchService(noninteractive=True)
service.include_launch_description(
    LaunchDescription([
        IncludeLaunchDescription(AnyLaunchDescriptionSource(sys.argv[1]))
    ])
)
raise SystemExit(service.run())
"""

    def __init__(self, node, launch_config, wait_cond='True', execute_wait=0.01):
        """Initialize the launch context."""
        super().__init__(node, execute_wait)
        Logger.initialize(node)

        self._launch_process = None
        self._launch_tempdir = None
        self._launch_file = self._resolve_launch_file(launch_config)
        self._wait_cond = wait_cond
        self._valid = self._launch_file is not None and os.path.isfile(self._launch_file)
        self._return_code = None
        self._stop_signal = None

    def _resolve_launch_file(self, launch_config):
        """Return a launch file path, materializing inline configs into a temp file when needed."""
        if launch_config.startswith('~') or launch_config.startswith('/'):
            return os.path.expanduser(launch_config)

        if os.path.isfile(launch_config):
            return os.path.abspath(launch_config)

        if '/' in launch_config and re.match(r'^[A-Za-z0-9_]+/.+\.(launch(\.(py|xml))?|py|xml)$', launch_config):
            try:
                pkgpath = get_package_share_directory(launch_config.split('/')[0])
            except PackageNotFoundError:
                return None
            return os.path.join(pkgpath, '/'.join(launch_config.split('/')[1:]))

        self._launch_tempdir = tempfile.mkdtemp(prefix='flexbe_launch_')
        stripped = launch_config.lstrip()
        suffix = '.launch.xml' if stripped.startswith('<') else '.launch.py'
        launch_file = os.path.join(self._launch_tempdir, f'generated{suffix}')
        with open(launch_file, 'w', encoding='utf-8') as handle:
            handle.write(launch_config)
            if not launch_config.endswith('\n'):
                handle.write('\n')
        return launch_file

    def __enter__(self):
        """Start the launch runner subprocess and wait for the launch condition."""
        if not self._valid:
            Logger.print_negative('launchfile is invalid or missing')
            return self

        self._launch_process = subprocess.Popen(
            [sys.executable, '-c', self._RUNNER_CODE, self._launch_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        Logger.print_positive('launchfile running')

        try:
            while self.ok():
                if bool(eval(self._wait_cond, self._evaluation_globals(), {})):
                    Logger.print_positive('waiting condition satisfied')
                    return self
                self.sleep()
            self._valid = False
            Logger.print_negative('waiting condition was not satisfied before launchfile stopped')
        except (AttributeError, NameError, SyntaxError, TypeError, ValueError) as e:
            self._valid = False
            Logger.print_negative('unable to check waiting condition:\n\t%s' % str(e))
        return self

    def ok(self):
        """Return ok status (default loop check)."""
        self._poll_process()
        return super().ok() and self._launch_process is not None and self._return_code is None

    def verify(self):
        """Verify valid."""
        return self._valid

    def spin_once(self):
        """Spin test runner loop once."""
        self._poll_process()
        self.sleep()

    def wait_for_finishing(self):
        """Wait for finishing."""
        self._node.get_logger().info('Waiting for all launched nodes to exit')
        while self._launch_process is not None and self._return_code is None and super().ok():
            self._poll_process()
            self.sleep()

    def __exit__(self, exception_type, exception_value, traceback):
        del exception_type, exception_value, traceback
        self._stop_launch_process()
        if self._launch_tempdir is not None:
            shutil.rmtree(self._launch_tempdir, ignore_errors=True)
            self._launch_tempdir = None
        Logger.print_positive('launchfile stopped')
        return False

    @property
    def success(self):
        """Verify success."""
        self._poll_process()
        return self._return_code in (None, 0) or self._stop_signal == signal.SIGINT

    def _evaluation_globals(self):
        """Return globals available to launch wait-condition expressions."""
        eval_rclpy = rclpy
        try:
            wait_for_message = __import__('rclpy.wait_for_message', fromlist=['wait_for_message']).wait_for_message
            eval_rclpy = self._EvalRclpyProxy(rclpy, wait_for_message=wait_for_message)
        except ImportError:
            pass
        return {
            '__builtins__': __builtins__,
            '__import__': __import__,
            'node': self._node,
            'os': os,
            'rclpy': eval_rclpy,
            'time': time,
        }

    def _poll_process(self):
        """Update cached subprocess return code."""
        if self._launch_process is None:
            return
        return_code = self._launch_process.poll()
        if return_code is None:
            return
        self._return_code = return_code
        if return_code != 0:
            self._valid = False

    def _stop_launch_process(self):
        """Terminate the launch subprocess and its process group."""
        if self._launch_process is None:
            return
        self._poll_process()
        if self._return_code is None:
            try:
                self._stop_signal = signal.SIGINT
                os.killpg(self._launch_process.pid, signal.SIGINT)
                self._launch_process.wait(timeout=5.0)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                self._stop_signal = signal.SIGTERM
                os.killpg(self._launch_process.pid, signal.SIGTERM)
                try:
                    self._launch_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self._stop_signal = signal.SIGKILL
                    os.killpg(self._launch_process.pid, signal.SIGKILL)
                    self._launch_process.wait(timeout=5.0)
            self._poll_process()
        self._launch_process = None


class LaunchPyTestContext(LaunchContext):
    """Launch-backed test context with the pytest timeout and loop limits."""

    def __init__(self, node, launch_config, wait_cond='True', timeout_sec=None, max_cnt=50, execute_wait=0.01):
        """Initialize the launch-backed pytest context."""
        super().__init__(node, launch_config, wait_cond=wait_cond, execute_wait=execute_wait)
        self._timeout_sec = float(timeout_sec) if timeout_sec is not None else None
        self._cnt = 0
        self._max_cnt = None
        self._time_out = None
        self._limits_active = False
        if max_cnt is not None:
            self._max_cnt = int(max_cnt)

        assert self._max_cnt is not None or self._timeout_sec is not None, 'Must have either timeout or max cnt set!'

    def __enter__(self):
        """Start the launch runner before enabling pytest loop limits."""
        super().__enter__()
        if self.verify():
            self._cnt = 0
            self._time_out = time.time() + self._timeout_sec if self._timeout_sec is not None else None
            self._limits_active = True
        return self

    def ok(self):
        """Return ok status based on launch health, time, and iteration count."""
        if not super().ok():
            return False

        if not self._limits_active:
            return True

        if self._time_out is not None and time.time() > self._time_out:
            return False

        self._cnt += 1
        if self._max_cnt is not None and self._cnt > self._max_cnt:
            return False

        return True
