#!/usr/bin/env python3

"""Focused tests for the onboard start wrapper."""

import unittest
from unittest.mock import patch

import flexbe_onboard.start_behavior as start_behavior


class _FakeLogger:

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class _FakeExecutor:

    def __init__(self, spin_exception=None):
        self._spin_exception = spin_exception
        self.added_nodes = []
        self.spin_calls = 0
        self.spin_once_calls = []

    def add_node(self, node):
        self.added_nodes.append(node)

    def spin(self):
        self.spin_calls += 1
        if self._spin_exception is not None:
            raise self._spin_exception

    def spin_once(self, timeout_sec=None):
        self.spin_once_calls.append(timeout_sec)


class _FakeOnboard:

    def __init__(self, behavior_shutdown_result=False, verify_results=None,
                 behavior_shutdown_exception=None, onboard_shutdown_exception=None,
                 destroy_exception=None):
        self._behavior_shutdown_result = behavior_shutdown_result
        self._verify_results = list(verify_results or [])
        self._behavior_shutdown_exception = behavior_shutdown_exception
        self._onboard_shutdown_exception = onboard_shutdown_exception
        self._destroy_exception = destroy_exception
        self._running = False
        self.executor = None
        self.logger = _FakeLogger()
        self.behavior_shutdown_calls = 0
        self.verify_calls = []
        self.onboard_shutdown_calls = 0
        self.destroy_calls = 0

    def get_logger(self):
        return self.logger

    def behavior_shutdown(self):
        self.behavior_shutdown_calls += 1
        if self._behavior_shutdown_exception is not None:
            raise self._behavior_shutdown_exception
        return self._behavior_shutdown_result

    def verify_no_active_behaviors(self, timeout=0.0):
        self.verify_calls.append(timeout)
        return self._verify_results.pop(0) if self._verify_results else False

    def onboard_shutdown(self):
        self.onboard_shutdown_calls += 1
        if self._onboard_shutdown_exception is not None:
            raise self._onboard_shutdown_exception

    def destroy_node(self):
        self.destroy_calls += 1
        if self._destroy_exception is not None:
            raise self._destroy_exception


class TestStartBehaviorMain(unittest.TestCase):
    """Cover the standalone onboard launcher wrapper."""

    def test_main_runs_clean_startup_and_shutdown_path(self):
        """Clean startup should spin once, shut down the node, and stop proxies."""
        onboard = _FakeOnboard(behavior_shutdown_result=False)
        executor = _FakeExecutor()

        with patch('flexbe_onboard.start_behavior.FlexbeOnboard', return_value=onboard), \
                patch('flexbe_onboard.start_behavior.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor) as executor_cls, \
                patch('flexbe_onboard.start_behavior.rclpy.init') as init_mock, \
                patch('flexbe_onboard.start_behavior.rclpy.try_shutdown') as try_shutdown_mock, \
                patch('flexbe_onboard.start_behavior.shutdown_proxies') as shutdown_proxies_mock, \
                patch('builtins.print') as print_mock:
            start_behavior.main(args=['--demo'])

        init_mock.assert_called_once()
        self.assertEqual(['--demo'], init_mock.call_args.kwargs['args'])
        executor_cls.assert_called_once_with()
        self.assertIs(onboard.executor, executor)
        self.assertEqual([onboard], executor.added_nodes)
        self.assertEqual(1, executor.spin_calls)
        self.assertEqual(1, onboard.behavior_shutdown_calls)
        self.assertEqual([], onboard.verify_calls)
        self.assertEqual(1, onboard.onboard_shutdown_calls)
        self.assertEqual(1, onboard.destroy_calls)
        shutdown_proxies_mock.assert_called_once_with()
        try_shutdown_mock.assert_called_once_with()
        self.assertEqual([0.001] * 180, executor.spin_once_calls)
        printed = '\n'.join(call.args[0] for call in print_mock.call_args_list if call.args)
        self.assertIn('All onboard behaviors are stopped', printed)
        self.assertIn('Done with behavior executive', printed)

    def test_main_handles_keyboard_interrupt_and_waits_for_behavior_cleanup(self):
        """A keyboard interrupt should trigger behavior cleanup and continue normal shutdown."""
        onboard = _FakeOnboard(behavior_shutdown_result=True, verify_results=[False, True])
        executor = _FakeExecutor(spin_exception=KeyboardInterrupt())

        with patch('flexbe_onboard.start_behavior.FlexbeOnboard', return_value=onboard), \
                patch('flexbe_onboard.start_behavior.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor), \
                patch('flexbe_onboard.start_behavior.rclpy.init'), \
                patch('flexbe_onboard.start_behavior.rclpy.try_shutdown'), \
                patch('flexbe_onboard.start_behavior.shutdown_proxies'), \
                patch('builtins.print') as print_mock:
            start_behavior.main()

        self.assertEqual(1, onboard.behavior_shutdown_calls)
        self.assertEqual([0.1, 0.1], onboard.verify_calls)
        self.assertEqual(280, len(executor.spin_once_calls))
        printed = '\n'.join(call.args[0] for call in print_mock.call_args_list if call.args)
        self.assertIn('Keyboard interrupt request', printed)
        self.assertIn('Active behavior still running onboard', printed)
        self.assertIn('onboard shutdown requested', printed)

    def test_main_reports_executor_shutdown_proxy_and_try_shutdown_exceptions(self):
        """Exception handlers should keep the wrapper progressing through all cleanup stages."""
        onboard = _FakeOnboard(behavior_shutdown_exception=ValueError('cleanup boom'))
        executor = _FakeExecutor(spin_exception=RuntimeError('spin boom'))

        with patch('flexbe_onboard.start_behavior.FlexbeOnboard', return_value=onboard), \
                patch('flexbe_onboard.start_behavior.rclpy.executors.SingleThreadedExecutor',
                      return_value=executor), \
                patch('flexbe_onboard.start_behavior.rclpy.init'), \
                patch('flexbe_onboard.start_behavior.rclpy.try_shutdown',
                      side_effect=RuntimeError('shutdown boom')), \
                patch('flexbe_onboard.start_behavior.shutdown_proxies',
                      side_effect=RuntimeError('proxy boom')), \
                patch('traceback.format_exc', return_value='traceback%%body'), \
                patch('builtins.print') as print_mock:
            start_behavior.main()

        printed = '\n'.join(call.args[0] for call in print_mock.call_args_list if call.args)
        self.assertIn('Exception in executor', printed)
        self.assertIn('Exception in onboard shutdown', printed)
        self.assertIn('Exception in onboard proxy and node shutdown', printed)
        self.assertIn('Exception from rclpy.shutdown for start onboard behavior', printed)


if __name__ == '__main__':
    unittest.main()
