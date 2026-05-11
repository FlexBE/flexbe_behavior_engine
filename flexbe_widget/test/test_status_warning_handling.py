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


"""Regression tests for WARNING status handling in widget paths."""

import threading
from types import SimpleNamespace
import unittest

from flexbe_msgs.msg import BEStatus

from flexbe_widget.behavior_action_server import BehaviorActionServer
from flexbe_widget.behavior_launcher import BehaviorLauncher

from rclpy.action import CancelResponse
from rclpy.action import GoalResponse


class _FakeLogger:

    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass

    def loginfo(self, *_args, **_kwargs):
        pass


class _FakeNode:

    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


class _FakeGoal:

    def __init__(self):
        self.is_active = True
        self.aborted = 0
        self.canceled_count = 0
        self.request = SimpleNamespace(behavior_name='demo', arg_keys=[], arg_values=[],
                                       input_keys=[], input_values=[])

    def abort(self):
        self.aborted += 1
        self.is_active = False

    def canceled(self):
        self.canceled_count += 1
        self.is_active = False

    def succeed(self):
        self.is_active = False

    def publish_feedback(self, _feedback):
        pass


class _FakePublisher:

    def __init__(self):
        self.count = 0

    def publish(self, _msg):
        self.count += 1


class TestWarningStatusHandling(unittest.TestCase):
    """Test WARNING status handling behavior in action server and launcher."""

    def test_action_server_warning_is_non_terminal(self):
        """Verify action server ignores WARNING because onboard no longer uses it as terminal."""
        server = object.__new__(BehaviorActionServer)
        server._node = _FakeNode()
        server._behavior_started = True
        server._active_behavior_id = 42
        server._current_state = 7
        server._current_goal = _FakeGoal()
        server.running = True
        server._preempt_requested = False
        server.outcome = ''

        msg = SimpleNamespace(code=BEStatus.WARNING, behavior_id=42, args=[])
        server._status_cb(msg)

        self.assertEqual(server._current_goal.aborted, 0)
        self.assertEqual(server.outcome, '')
        self.assertTrue(server.running)
        self.assertTrue(server._behavior_started)

    def test_launcher_warning_leaves_ready_event_unchanged(self):
        """Verify launcher ignores WARNING because onboard no longer uses it as terminal."""
        launcher = object.__new__(BehaviorLauncher)
        launcher._ready_event = threading.Event()
        launcher._ready_event.set()
        logger = _FakeLogger()
        launcher.get_logger = lambda: logger

        msg = SimpleNamespace(code=BEStatus.WARNING)
        launcher._status_callback(msg)

        self.assertTrue(launcher._ready_event.is_set())

    def test_launcher_started_sets_ready_event(self):
        """Verify launcher allows follow-up switch/update requests once execution has started."""
        launcher = object.__new__(BehaviorLauncher)
        launcher._ready_event = threading.Event()
        logger = _FakeLogger()
        launcher.get_logger = lambda: logger

        msg = SimpleNamespace(code=BEStatus.STARTED)
        launcher._status_callback(msg)

        self.assertTrue(launcher._ready_event.is_set())

    def test_launcher_running_sets_ready_event(self):
        """Verify launcher stays open for switch/update requests while a behavior is active."""
        launcher = object.__new__(BehaviorLauncher)
        launcher._ready_event = threading.Event()
        logger = _FakeLogger()
        launcher.get_logger = lambda: logger

        msg = SimpleNamespace(code=BEStatus.RUNNING)
        launcher._status_callback(msg)

        self.assertTrue(launcher._ready_event.is_set())

    def test_launcher_switching_clears_ready_event(self):
        """Verify launcher blocks new requests while onboard is in the middle of a switch."""
        launcher = object.__new__(BehaviorLauncher)
        launcher._ready_event = threading.Event()
        launcher._ready_event.set()
        logger = _FakeLogger()
        launcher.get_logger = lambda: logger

        msg = SimpleNamespace(code=BEStatus.SWITCHING)
        launcher._status_callback(msg)

        self.assertFalse(launcher._ready_event.is_set())

    def test_action_server_preempt_callback_accepts_cancel(self):
        """Verify action server returns ACCEPT when preempt is requested."""
        server = object.__new__(BehaviorActionServer)
        server._node = _FakeNode()
        server._preempt_requested = False
        server._behavior_started = True
        server._preempt_pub = _FakePublisher()

        response = server._preempt_cb(_FakeGoal())

        self.assertEqual(CancelResponse.ACCEPT, response)
        self.assertTrue(server._preempt_requested)
        self.assertEqual(1, server._preempt_pub.count)

    def test_action_server_rejects_new_goal_while_running(self):
        """Verify a second goal does not replace the active one while running."""
        server = object.__new__(BehaviorActionServer)
        server._node = _FakeNode()
        current_goal = _FakeGoal()
        server._preempt_requested = False
        server._current_goal = current_goal
        server.running = True

        response = server._goal_request_cb(_FakeGoal())

        self.assertIs(current_goal, server._current_goal)
        self.assertEqual(GoalResponse.REJECT, response)


if __name__ == '__main__':
    unittest.main()
