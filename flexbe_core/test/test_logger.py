#!/usr/bin/env python3

# Copyright 2024 Christopher Newport University
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


"""Test FlexBE Exception handling."""
import time
import unittest

from flexbe_core import EventState, OperatableStateMachine, initialize_flexbe_core
from flexbe_core.logger import Logger
from flexbe_core.proxy import shutdown_proxies

import rclpy
from rclpy.executors import MultiThreadedExecutor


class TestLogger(unittest.TestCase):
    """Test FlexBE Logger handling."""

    test = 0
    __EXECUTE_TIMEOUT_SEC = 0.2  # 0.025  # Timeout in executor loops for spin once
    __TIME_SLEEP = 0.2  # 0.025  # Sleep time for loops

    def __init__(self, *args, **kwargs):
        """Initialize TestLogger instance."""
        super().__init__(*args, **kwargs)

    def setUp(self):
        """Set up the test."""
        TestLogger.test += 1
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('logger_test_' + str(self.test), context=self.context)
        self.node.get_logger().info(' set up logger test %d (%d) ... ' % (self.test, self.context.ok()))
        self.executor.add_node(self.node)
        initialize_flexbe_core(self.node)

    def tearDown(self):
        """Tear down the test."""
        self.node.get_logger().info(' shutting down logger test %d (%d) ... ' % (self.test, self.context.ok()))
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestLogger.__EXECUTE_TIMEOUT_SEC)

        self.node.get_logger().info('    shutting down proxies in logger test %d ... ' % (self.test))
        shutdown_proxies()
        time.sleep(TestLogger.__TIME_SLEEP)

        self.node.get_logger().info('    destroy node in core test %d ... ' % (self.test))
        self.node.destroy_node()

        time.sleep(TestLogger.__TIME_SLEEP)
        self.executor.shutdown()
        time.sleep(TestLogger.__TIME_SLEEP)

        # Kill it with fire to make sure not stray published topics are available
        rclpy.shutdown(context=self.context)
        time.sleep(TestLogger.__TIME_SLEEP * 2)

    def test_throttle_logger_one(self):
        """Test throttle logger one."""
        self.node.get_logger().info('test_throttle_logger_one ...')
        self.node.declare_parameter('max_throttle_logging_size', 100)
        self.node.declare_parameter('throttle_logging_clear_ratio', 0.25)
        initialize_flexbe_core(self.node)  # Update the logger node

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestLogger.__EXECUTE_TIMEOUT_SEC)

        class ThrottleSingleLog(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'])
                self._trials = Logger.MAX_LAST_LOGGED_SIZE * 2
                Logger.logerr_throttle(0.0, 'test')

            def execute(self, userdata):
                Logger.logerr_throttle(0.0, 'test')
                self._trials -= 1
                if self._trials == 0:
                    return 'done'

        state_instance = ThrottleSingleLog()
        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', state_instance, transitions={'done': 'done'})
        outcome = None
        self.assertEqual(Logger.MAX_LAST_LOGGED_SIZE, 100)  # default size
        self.assertAlmostEqual(Logger.LAST_LOGGED_CLEARING_RATIO, 0.25)  # default ratio
        self.assertEqual(state_instance._trials, Logger.MAX_LAST_LOGGED_SIZE * 2)
        while outcome is None:
            outcome = sm.execute(None)
            self.assertEqual(len(Logger._last_logged), 1)
        self.assertEqual(outcome, 'done')
        self.assertEqual(state_instance._trials, 0)

        self.assertIsNone(sm._last_exception)
        self.node.get_logger().info('test_throttle_logger_one  - OK! ')

    def test_throttle_logger_err_multi(self):
        """Test throttle logger with errors."""
        self.node.get_logger().info('test_throttle_logger_err_multi ...')
        self.node.declare_parameter('max_throttle_logging_size', 200)
        self.node.declare_parameter('throttle_logging_clear_ratio', 0.35)
        initialize_flexbe_core(self.node)  # Update the logger node

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestLogger.__EXECUTE_TIMEOUT_SEC)

        class ThrottleMultiLog(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'])
                self._trials = Logger.MAX_LAST_LOGGED_SIZE * 2
                Logger.logerr_throttle(0.01, '0_test')

            def execute(self, userdata):
                Logger.logerr_throttle(0.01, f'{self._trials}_test')
                self._trials -= 1
                if self._trials == 0:
                    return 'done'

        state_instance = ThrottleMultiLog()
        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', state_instance, transitions={'done': 'done'})
        outcome = None
        self.assertEqual(Logger.MAX_LAST_LOGGED_SIZE, 200)  # default size
        self.assertAlmostEqual(Logger.LAST_LOGGED_CLEARING_RATIO, 0.35)  # default ratio
        self.assertEqual(state_instance._trials, Logger.MAX_LAST_LOGGED_SIZE * 2)
        while outcome is None:
            outcome = sm.execute(None)
            self.assertTrue(1 < len(Logger._last_logged) <= Logger.MAX_LAST_LOGGED_SIZE)
        self.assertEqual(outcome, 'done')
        self.assertEqual(state_instance._trials, 0)

        self.assertIsNone(sm._last_exception)
        self.node.get_logger().info('test_throttle_logger_err_multi  - OK! ')

    def test_throttle_logger_multiple_params(self):
        """Test throttle logger with multiple params."""
        self.node.get_logger().info('test_throttle_logger_multiple_params ...')
        self.node.declare_parameter('max_throttle_logging_size', 100)
        self.node.declare_parameter('throttle_logging_clear_ratio', 0.7)

        initialize_flexbe_core(self.node)  # Update the logger node

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestLogger.__EXECUTE_TIMEOUT_SEC)

        class ThrottleMultiLog(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'])
                self._trials = Logger.MAX_LAST_LOGGED_SIZE * 2
                Logger.logerr_throttle(0.01, '0_test')

            def execute(self, userdata):
                Logger.logerr(f'{self._trials}_test')
                Logger.logerr_throttle(0.0, f'{self._trials}_test')
                Logger.logwarn_throttle(0.0, f'{self._trials}_test')
                Logger.loginfo_throttle(0.0, f'{self._trials}_test')
                Logger.loghint_throttle(0.0, f'{self._trials}_test')
                self._trials -= 1
                if self._trials == 0:
                    return 'done'

        state_instance = ThrottleMultiLog()
        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', state_instance, transitions={'done': 'done'})
        outcome = None
        self.assertEqual(state_instance._trials, Logger.MAX_LAST_LOGGED_SIZE * 2)
        self.assertEqual(Logger.MAX_LAST_LOGGED_SIZE, 100)  # parameterized size
        self.assertAlmostEqual(Logger.LAST_LOGGED_CLEARING_RATIO, 0.7)  # parameterized
        while outcome is None:
            outcome = sm.execute(None)
            self.assertTrue(1 < len(Logger._last_logged) <= Logger.MAX_LAST_LOGGED_SIZE)
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestLogger.__EXECUTE_TIMEOUT_SEC * 0.5)
        self.assertEqual(outcome, 'done')
        self.assertEqual(state_instance._trials, 0)

        self.assertIsNone(sm._last_exception)
        self.node.get_logger().info('test_throttle_logger_multiple  - OK! ')

    def test_throttle_logger_multiple(self):
        """Test throttle logger multiple."""
        self.node.get_logger().info('test_throttle_logger_multiple_params ...')
        self.node.declare_parameter('max_throttle_logging_size', 120)
        self.node.declare_parameter('throttle_logging_clear_ratio', 0.22)
        initialize_flexbe_core(self.node)  # Update the logger node

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestLogger.__EXECUTE_TIMEOUT_SEC)

        class ThrottleMultiLog(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'])
                self._trials = Logger.MAX_LAST_LOGGED_SIZE * 2
                Logger.logerr_throttle(0.01, '0_test')

            def execute(self, userdata):
                Logger.logerr(f'{self._trials}_test')
                Logger.logerr_throttle(0.0, f'{self._trials}_test')
                Logger.logwarn_throttle(0.0, f'{self._trials}_test')
                Logger.loginfo_throttle(0.0, f'{self._trials}_test')
                Logger.loghint_throttle(0.0, f'{self._trials}_test')
                self._trials -= 1
                if self._trials == 0:
                    return 'done'

        state_instance = ThrottleMultiLog()
        sm = OperatableStateMachine(outcomes=['done'])
        with sm:
            OperatableStateMachine.add('state', state_instance, transitions={'done': 'done'})
        outcome = None
        self.assertEqual(state_instance._trials, Logger.MAX_LAST_LOGGED_SIZE * 2)
        self.assertEqual(Logger.MAX_LAST_LOGGED_SIZE, 120)  # default size
        self.assertAlmostEqual(Logger.LAST_LOGGED_CLEARING_RATIO, 0.22)  # default ratio
        while outcome is None:
            outcome = sm.execute(None)
            self.assertTrue(1 < len(Logger._last_logged) <= Logger.MAX_LAST_LOGGED_SIZE)
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestLogger.__EXECUTE_TIMEOUT_SEC * 0.5)
        self.assertEqual(outcome, 'done')
        self.assertEqual(state_instance._trials, 0)

        self.assertIsNone(sm._last_exception)
        self.node.get_logger().info('test_throttle_logger_multiple_params  - OK! ')


if __name__ == '__main__':
    unittest.main()
