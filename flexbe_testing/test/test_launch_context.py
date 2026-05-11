#!/usr/bin/env python3

# Copyright 2026  Christopher Newport University
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

"""Regression tests for ROS 2 LaunchContext support."""

import os
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from flexbe_testing.test_context import LaunchContext, LaunchPyTestContext
from flexbe_testing.tester import Tester

from rclpy.exceptions import ParameterNotDeclaredException


class _FakeParameterValue:

    def __init__(self, value):
        self.bool_value = value


class _FakeParameter:

    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        return _FakeParameterValue(self._value)


class _FakeLogger:

    def info(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass

    def debug(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class _FakeNode:

    def __init__(self):
        self._logger = _FakeLogger()
        self._parameters = {}
        self.context = object()

    def get_logger(self):
        return self._logger

    def get_parameter(self, name):
        if name not in self._parameters:
            raise ParameterNotDeclaredException(name)
        return _FakeParameter(self._parameters[name])

    def declare_parameter(self, name, value):
        self._parameters[name] = value
        return _FakeParameter(value)


class _FakeTestInterface:

    @staticmethod
    def instantiate(_params):
        return None

    @staticmethod
    def execute(_userdata, _context, spin_cb=None):
        if spin_cb is not None:
            spin_cb()
        return 'done'


class _FakeProcess:

    def __init__(self, poll_sequence=None, wait_results=None):
        self.pid = 12345
        self._poll_sequence = list(poll_sequence or [None])
        self._wait_results = list(wait_results or [0])
        self.wait_calls = []

    def poll(self):
        if len(self._poll_sequence) > 1:
            return self._poll_sequence.pop(0)
        return self._poll_sequence[0]

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        result = self._wait_results.pop(0) if self._wait_results else 0
        if isinstance(result, BaseException):
            raise result
        self._poll_sequence = [result]
        return result


class TestLaunchContext(unittest.TestCase):
    """Validate inline ROS 2 launch execution paths."""

    @patch('flexbe_testing.test_context.Logger.print_positive')
    @patch('flexbe_testing.test_context.Logger.print_negative')
    def test_resolves_package_launch_file(self, print_negative, _print_positive):
        """Package-relative launch paths should resolve via the package share directory."""
        with tempfile.TemporaryDirectory() as temp_dir:
            launch_dir = os.path.join(temp_dir, 'launch')
            os.makedirs(launch_dir)
            launch_file = os.path.join(launch_dir, 'demo.launch.py')
            with open(launch_file, 'w', encoding='utf-8') as handle:
                handle.write('def generate_launch_description():\n    return None\n')

            with patch('flexbe_testing.test_context.get_package_share_directory', return_value=temp_dir):
                context = LaunchContext(_FakeNode(), 'demo_pkg/launch/demo.launch.py')

            self.assertTrue(context.verify())
            self.assertEqual(launch_file, context._launch_file)
            print_negative.assert_not_called()

    @patch('flexbe_testing.test_context.Logger.print_positive')
    @patch('flexbe_testing.test_context.Logger.print_negative')
    def test_materializes_inline_launch_and_cleans_up_tempdir(self, _print_negative, _print_positive):
        """Inline launch snippets should be written to a temporary launch file and cleaned up."""
        context = LaunchContext(
            _FakeNode(),
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n',
        )

        self.assertTrue(context.verify())
        self.assertTrue(os.path.isfile(context._launch_file))
        self.assertTrue(os.path.isdir(context._launch_tempdir))
        tempdir = context._launch_tempdir

        context.__exit__(None, None, None)

        self.assertFalse(os.path.exists(tempdir))

    @patch('flexbe_testing.test_context.rclpy.ok', return_value=True)
    @patch('flexbe_testing.test_context.Logger.print_positive')
    @patch('flexbe_testing.test_context.Logger.print_negative')
    @patch('flexbe_testing.test_context.subprocess.Popen')
    def test_enter_marks_invalid_when_launch_exits_before_wait_condition(
        self, popen, print_negative, _print_positive, _rclpy_ok
    ):
        """Fail verification when the launch process exits before the wait condition."""
        popen.return_value = _FakeProcess(poll_sequence=[1])
        context = LaunchContext(
            _FakeNode(),
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n',
            wait_cond='False',
            execute_wait=0.0,
        )

        context.__enter__()

        self.assertFalse(context.verify())
        print_negative.assert_called_with(
            'waiting condition was not satisfied before launchfile stopped'
        )

    @patch('flexbe_testing.test_context.os.killpg')
    @patch('flexbe_testing.test_context.rclpy.ok', return_value=True)
    @patch('flexbe_testing.test_context.Logger.print_positive')
    @patch('flexbe_testing.test_context.Logger.print_negative')
    @patch('flexbe_testing.test_context.subprocess.Popen')
    def test_tester_run_test_uses_real_launch_context(
        self, popen, _print_negative, _print_positive, _rclpy_ok, killpg
    ):
        """Execute launch-backed configs through LaunchContext without the old TODO."""
        popen.return_value = _FakeProcess(poll_sequence=[None], wait_results=[0])
        tester = Tester(_FakeNode(), executor=None, execute_wait=0.0)
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
            'launch': 'from launch import LaunchDescription\n'
                      'def generate_launch_description():\n'
                      '    return LaunchDescription([])\n',
            'wait_cond': 'True',
        }

        with patch('flexbe_testing.tester.Logger.print_positive'), \
                patch('flexbe_testing.tester.Logger.print_negative'), \
                patch('flexbe_testing.tester.Logger.print_result'), \
                patch('flexbe_testing.tester.Logger.print_title'), \
                patch('flexbe_testing.tester.DataProvider') as data_provider_cls, \
                patch.object(Tester, 'import_interface', return_value=_FakeTestInterface()):
            data_provider_cls.return_value.parse.side_effect = lambda value: value
            success = tester.run_test('demo', config)

        self.assertEqual(1, success)
        self.assertTrue(popen.called)
        killpg.assert_called_once()

    @patch('flexbe_testing.test_context.os.killpg')
    @patch('flexbe_testing.test_context.Logger.print_positive')
    @patch('flexbe_testing.test_context.Logger.print_negative')
    def test_success_treats_intentional_context_shutdown_as_clean(
        self, _print_negative, _print_positive, killpg
    ):
        """Consider an intentional context shutdown clean for launch-success checks."""
        context = LaunchContext(
            _FakeNode(),
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n',
        )
        context._launch_process = _FakeProcess(poll_sequence=[None], wait_results=[-2])

        context.__exit__(None, None, None)

        self.assertTrue(context.success)
        killpg.assert_called_once()

    @patch('flexbe_testing.test_context.os.killpg')
    @patch('flexbe_testing.test_context.Logger.print_positive')
    @patch('flexbe_testing.test_context.Logger.print_negative')
    def test_success_reports_forced_sigterm_shutdown_as_failure(
        self, _print_negative, _print_positive, killpg
    ):
        """Do not treat a launch that ignored SIGINT and required SIGTERM as clean."""
        context = LaunchContext(
            _FakeNode(),
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n',
        )
        fake_process = _FakeProcess(
            poll_sequence=[None, None, -15],
            wait_results=[subprocess.TimeoutExpired(cmd='launch', timeout=5.0), -15],
        )
        context._launch_process = fake_process

        context.__exit__(None, None, None)

        self.assertFalse(context.success)
        self.assertEqual([5.0, 5.0], fake_process.wait_calls)
        self.assertEqual(signal.SIGTERM, context._stop_signal)
        self.assertEqual(2, killpg.call_count)

    @patch('rclpy.wait_for_message.wait_for_message', return_value=('msg', None))
    def test_evaluation_globals_expose_wait_for_message_as_callable(self, wait_for_message_mock):
        """Allow wait conditions to call rclpy.wait_for_message(...) directly."""
        context = LaunchContext(
            _FakeNode(),
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n',
        )

        result = eval(
            "rclpy.wait_for_message('/demo', object)",
            context._evaluation_globals(),
            {},
        )

        self.assertEqual(('msg', None), result)
        wait_for_message_mock.assert_called_once_with('/demo', object)

    @patch('flexbe_testing.test_context.time.time', side_effect=[5.0, 5.2, 5.8])
    @patch.object(LaunchContext, '__enter__', autospec=True)
    @patch.object(LaunchContext, 'ok', autospec=True, return_value=True)
    def test_launch_pytest_context_starts_timeout_after_launch_is_ready(
        self, _launch_ok, launch_enter, _time_time
    ):
        """Start pytest timeout accounting after launch startup completes."""
        launch_enter.side_effect = lambda context: context

        context = LaunchPyTestContext(
            _FakeNode(),
            'demo_pkg/launch/demo.launch.py',
            timeout_sec=0.5,
            max_cnt=None,
            execute_wait=0.0,
        )
        context._valid = True

        context.__enter__()

        self.assertTrue(context.ok())
        self.assertFalse(context.ok())


if __name__ == '__main__':
    unittest.main()
