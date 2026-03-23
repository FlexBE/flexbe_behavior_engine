#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Christopher Newport University nor the names of its
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

"""Unit tests for logger lifecycle helpers."""

import logging
import unittest
from unittest.mock import patch

from flexbe_core import EventState
from flexbe_core.logger import Logger
from flexbe_core.state_logger import PublishBehaviorLogMessage, StateLogger

from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.logging import LoggingSeverity


class _FakeRosLogger:
    """Minimal ROS logger interface used by Logger.initialize()."""

    def __init__(self):
        self.name = 'fake_ros_logger'
        self.info_messages = []
        self.warning_messages = []
        self.enabled_checks = []
        self.enabled_severities = {
            LoggingSeverity.INFO,
            LoggingSeverity.WARN,
            LoggingSeverity.ERROR,
            LoggingSeverity.DEBUG
        }

    def info(self, msg):
        """Record info logging in tests."""
        self.info_messages.append(msg)

    def warning(self, msg):
        """Record warning logging in tests."""
        self.warning_messages.append(msg)

    def debug(self, _msg):
        """Ignore debug logging in tests."""

    def error(self, _msg):
        """Ignore error logging in tests."""

    def is_enabled_for(self, severity):
        """Return whether a severity is enabled."""
        self.enabled_checks.append(severity)
        return severity in self.enabled_severities


class _FakeNode:
    """Minimal node exposing publisher lifecycle hooks."""

    def __init__(self):
        self.created_publishers = []
        self.destroyed_publishers = []
        self._logger = _FakeRosLogger()

    def create_publisher(self, msg_type, topic, queue_size):
        """Create and record a fake publisher object."""
        publisher = object()
        self.created_publishers.append((publisher, msg_type, topic, queue_size))
        return publisher

    def destroy_publisher(self, publisher):
        """Record destroyed publishers."""
        self.destroyed_publishers.append(publisher)
        return True

    def get_parameter(self, _name):
        """Raise as though optional logger parameters were not declared."""
        raise ParameterNotDeclaredException('missing')

    def get_logger(self):
        """Return the fake ROS logger."""
        return self._logger

    def get_clock(self):
        """Return a monotonic fake ROS clock."""
        class _Clock:

            def __init__(self):
                self._count = 0

            def now(self):
                from rclpy.time import Time
                self._count += 1
                return Time(nanoseconds=self._count)

        if not hasattr(self, '_clock'):
            self._clock = _Clock()
        return self._clock

    @property
    def clock_calls(self):
        """Return how many times the fake clock has been sampled."""
        if not hasattr(self, '_clock'):
            return 0
        return self._clock._count


class TestLoggerLifecycle(unittest.TestCase):
    """Validate logger singleton lifecycle behavior."""

    def tearDown(self):
        """Reset logger singleton state after each test."""
        Logger.shutdown()

    def test_initialize_reuses_existing_publisher_for_same_node(self):
        """Repeated initialization on the same node should not create duplicate publishers."""
        node = _FakeNode()
        Logger.initialize(node)
        existing_pub = Logger._pub

        Logger.initialize(node)

        self.assertIs(Logger._pub, existing_pub)
        self.assertEqual(len(node.created_publishers), 1)

    def test_initialize_replaces_publisher_for_new_node(self):
        """Initializing on a different node should destroy the old publisher first."""
        first_node = _FakeNode()
        second_node = _FakeNode()

        Logger.initialize(first_node)
        first_pub = Logger._pub
        Logger.initialize(second_node)

        self.assertEqual(first_node.destroyed_publishers, [first_pub])
        self.assertEqual(len(second_node.created_publishers), 1)

    def test_shutdown_releases_logger_publisher(self):
        """Logger shutdown should destroy the active publisher and reset singleton state."""
        node = _FakeNode()
        Logger.initialize(node)
        publisher = Logger._pub

        Logger.shutdown()

        self.assertEqual(node.destroyed_publishers, [publisher])
        self.assertIsNone(Logger._pub)
        self.assertIsNone(Logger._node)

    def test_localinfo_throttle_reuses_last_logged_window(self):
        """Local throttle should suppress repeated terminal info logs in the throttle window."""
        node = _FakeNode()
        Logger.initialize(node)

        with patch('flexbe_core.logger._rclpy.rclpy_logging_rcutils_log') as log_call:
            Logger.localinfo_throttle(10.0, 'throttled info')
            Logger.localinfo_throttle(10.0, 'throttled info')

        self.assertEqual(log_call.call_count, 1)
        self.assertEqual(log_call.call_args[0][2], 'throttled info')

    def test_localinfo_skips_string_formatting_when_info_disabled(self):
        """Local info should not format messages when INFO severity is disabled."""
        node = _FakeNode()
        node.get_logger().enabled_severities.discard(LoggingSeverity.INFO)
        Logger.initialize(node)

        class _Unformattable:

            def __str__(self):
                raise AssertionError('INFO formatting should have been skipped')

        Logger.localinfo('disabled info %s', _Unformattable())

        self.assertEqual(node.get_logger().info_messages, [])

    def test_check_local_enabled_caches_severity_flags_for_local_logs(self):
        """Local logging should reuse cached severity flags until explicitly refreshed."""
        node = _FakeNode()
        Logger.initialize(node)
        node.get_logger().enabled_checks.clear()

        with patch('flexbe_core.logger._rclpy.rclpy_logging_rcutils_log') as log_call:
            Logger.localinfo('first')
            Logger.localinfo('second')

        self.assertEqual(log_call.call_count, 2)
        self.assertEqual(node.get_logger().enabled_checks, [])

        node.get_logger().enabled_severities.discard(LoggingSeverity.INFO)
        Logger.check_local_enabled()
        node.get_logger().enabled_checks.clear()

        with patch('flexbe_core.logger._rclpy.rclpy_logging_rcutils_log') as log_call:
            Logger.localinfo('disabled')

        self.assertEqual(log_call.call_count, 0)
        self.assertEqual(node.get_logger().enabled_checks, [])

    def test_state_logger_shutdown_closes_only_flexbe_handlers(self):
        """State logger shutdown should close its own handlers without global logging shutdown."""
        closed_handlers = []
        kept_handler = logging.StreamHandler()
        flexbe_logger = logging.getLogger('flexbe')
        other_logger = logging.getLogger('external')
        original_flexbe_handlers = list(flexbe_logger.handlers)
        original_other_handlers = list(other_logger.handlers)

        class _ClosableHandler(logging.Handler):

            def emit(self, record):
                return None

            def close(self):
                closed_handlers.append(self)
                super().close()

        handler = _ClosableHandler()
        flexbe_logger.addHandler(handler)
        other_logger.addHandler(kept_handler)
        StateLogger.enabled = True
        try:
            StateLogger.shutdown()
            self.assertIn(handler, closed_handlers)
            self.assertIn(kept_handler, other_logger.handlers)
        finally:
            for existing in list(flexbe_logger.handlers):
                flexbe_logger.removeHandler(existing)
            for existing in list(other_logger.handlers):
                other_logger.removeHandler(existing)
            for existing in original_flexbe_handlers:
                flexbe_logger.addHandler(existing)
            for existing in original_other_handlers:
                other_logger.addHandler(existing)
            StateLogger.enabled = False

    def test_publish_behavior_log_handler_releases_proxy_publisher_on_close(self):
        """Closing the state-log publish handler should remove its proxy publisher."""
        removed_topics = []

        class _FakeProxyPublisher:

            def __init__(self, topics):
                self.topics = topics

            def publish(self, topic, msg):
                return None

            def remove_publisher(self, topic):
                removed_topics.append(topic)

        with patch('flexbe_core.state_logger.ProxyPublisher', _FakeProxyPublisher):
            handler = PublishBehaviorLogMessage(topic='/state_logger')
            handler.close()

        self.assertEqual(removed_topics, ['/state_logger'])

    def test_state_logger_wrappers_are_installed_once_per_class(self):
        """Repeated state instances should share the same wrapped methods from class-level installation."""

        class _LoggedState(EventState):

            def __init__(self):
                super().__init__(outcomes=['done'], input_keys=['data'])

            def execute(self, userdata):
                return 'done'

            def on_enter(self, userdata):
                return None

        first = _LoggedState()
        second = _LoggedState()

        self.assertIs(first.execute.__func__, second.execute.__func__)
        self.assertIs(first.on_enter.__func__, second.on_enter.__func__)

    def test_state_logger_skips_event_and_outcome_work_when_info_disabled(self):
        """State logger wrappers should skip clock and payload work when INFO logging is disabled."""
        node = _FakeNode()
        StateLogger._node = node
        StateLogger.enabled = True

        class _FakeStateLogger:

            def isEnabledFor(self, _level):
                return False

            def info(self, _msg):
                raise AssertionError('INFO logging should have been skipped')

        with patch.object(StateLogger, 'get', return_value=_FakeStateLogger()):

            @StateLogger.log_outcomes('state_test')
            @StateLogger.log_events('state_test', enter='on_enter')
            class _LoggedState:

                def __init__(self):
                    self.name = 'logged'
                    self.path = '/logged'

                def execute(self, userdata):
                    return 'done'

                def on_enter(self, userdata):
                    return None

            state = _LoggedState()
            state.on_enter(None)
            self.assertEqual(state.execute(None), 'done')

        self.assertEqual(node.clock_calls, 0)

    def test_state_logger_userdata_uses_instance_input_keys(self):
        """Userdata logging should honor the state instance input_keys at call time."""
        node = _FakeNode()
        StateLogger._node = node
        StateLogger.enabled = True
        logged_records = []

        class _FakeStateLogger:

            def isEnabledFor(self, level):
                return level == logging.DEBUG

            def debug(self, payload):
                logged_records.append(payload)

        with patch.object(StateLogger, 'get', return_value=_FakeStateLogger()):

            @StateLogger.log_userdata('state_test')
            class _LoggedState:

                def __init__(self, input_keys):
                    self.name = 'logged'
                    self.path = '/logged'
                    self.input_keys = input_keys

                def on_enter(self, userdata):
                    return None

            first = _LoggedState(['first'])
            second = _LoggedState(['second'])

            first.on_enter({'first': 1, 'second': 2})
            second.on_enter({'first': 3, 'second': 4})

        self.assertEqual(logged_records[0]['userdata'], {'first': '1\n...\n'})
        self.assertEqual(logged_records[1]['userdata'], {'second': '4\n...\n'})


if __name__ == '__main__':
    unittest.main()
