#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
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

"""Unit tests for ProxyActionClient send_goal behavior."""

import unittest
from unittest.mock import patch

from action_msgs.msg import GoalStatus

from flexbe_core.core.exceptions import ProxyAvailabilityError
from flexbe_core.proxy.proxy_action_client import ProxyActionClient


class _FakeGoal:
    """Minimal fake action goal type."""

    __slots__ = ()


class _FakeActionType:
    """Minimal fake action type with Goal class."""

    Goal = _FakeGoal


class _FakeFuture:
    """Minimal future supporting callback registration."""

    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        """Record callbacks added by send_goal."""
        self.callbacks.append(callback)

    def done(self):
        return False


class _BrokenFuture(_FakeFuture):
    """Fake future that fails during callback registration."""

    def add_done_callback(self, _callback):
        """Simulate a callback registration failure."""
        raise RuntimeError('cannot register callback')


class _ImmediateFuture(_FakeFuture):
    """Fake future that invokes callbacks immediately."""

    def __init__(self, result_obj):
        super().__init__()
        self._result_obj = result_obj

    def add_done_callback(self, callback):
        """Record and immediately invoke callback as rclpy may do for completed futures."""
        self.callbacks.append(callback)
        callback(self)

    def done(self):
        return True

    def result(self):
        return self._result_obj


class _ImmediateResultResponse:
    """Minimal action result response container."""

    def __init__(self, result='ok', status=GoalStatus.STATUS_SUCCEEDED):
        self.result = result
        self.status = status


class _ImmediateGoalHandle:
    """Accepted goal handle that returns an already-complete result future."""

    accepted = True

    @staticmethod
    def get_result_async():
        """Return an already-complete result future."""
        return _ImmediateFuture(_ImmediateResultResponse())


class _FakeClient:
    """Minimal action client used to test send_goal call flow."""

    def __init__(self):
        self._action_type = _FakeActionType
        self.sent_goals = []

    def wait_for_server(self, *_args, **_kwargs):
        """Fail if called directly from send_goal path."""
        raise AssertionError('send_goal should not call wait_for_server() directly')

    def send_goal_async(self, goal, feedback_callback=None):
        """Capture sent goal and return fake future."""
        self.sent_goals.append((goal, feedback_callback))
        return _FakeFuture()


class _FailingClient(_FakeClient):
    """Fake action client that fails during async send."""

    def send_goal_async(self, goal, feedback_callback=None):
        """Capture the send attempt and fail before returning a future."""
        self.sent_goals.append((goal, feedback_callback))
        raise RuntimeError('send failed')


class _BrokenCallbackClient(_FakeClient):
    """Fake action client returning a future with broken callback registration."""

    def send_goal_async(self, goal, feedback_callback=None):
        """Capture sent goal and return a broken future."""
        self.sent_goals.append((goal, feedback_callback))
        return _BrokenFuture()


class _ImmediateDoneClient(_FakeClient):
    """Fake action client that returns an already-complete goal future."""

    def send_goal_async(self, goal, feedback_callback=None):
        """Capture sent goal and return an already-complete future."""
        self.sent_goals.append((goal, feedback_callback))
        return _ImmediateFuture(_ImmediateGoalHandle())


class TestProxyActionClientSendGoal(unittest.TestCase):
    """Validate send_goal behavior around availability and async dispatch."""

    def setUp(self):
        """Reset static proxy state before each test."""
        ProxyActionClient._clients.clear()
        ProxyActionClient._result.clear()
        ProxyActionClient._result_status.clear()
        ProxyActionClient._feedback.clear()
        ProxyActionClient._has_active_goal.clear()
        ProxyActionClient._current_goal.clear()
        ProxyActionClient._client_generation_counter = 0

    def test_send_goal_does_not_call_unbounded_wait_for_server(self):
        """send_goal should rely on availability check and dispatch async goal directly."""
        topic = '/action'
        fake_client = _FakeClient()
        ProxyActionClient._clients[topic] = {'client': fake_client, 'generation': 1, 'count': 1}

        with patch.object(ProxyActionClient, '_check_topic_available', return_value=True):
            ProxyActionClient.send_goal(topic, _FakeGoal(), wait_duration=0.01)

        self.assertEqual(len(fake_client.sent_goals), 1)
        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_UNKNOWN)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertIsNotNone(ProxyActionClient._current_goal[topic])

    def test_send_goal_default_wait_duration_is_fail_fast(self):
        """send_goal should pass the default 0.0 wait through to availability checks."""
        topic = '/action'
        fake_client = _FakeClient()
        ProxyActionClient._clients[topic] = {'client': fake_client, 'generation': 1, 'count': 1}

        with patch.object(ProxyActionClient, '_check_topic_available', return_value=True) as check_available:
            ProxyActionClient.send_goal(topic, _FakeGoal())

        check_available.assert_called_once_with(topic, wait_duration=0.0)
        self.assertEqual(len(fake_client.sent_goals), 1)

    def test_send_goal_rejects_second_goal_while_topic_is_active(self):
        """send_goal should fail fast if a goal is already active on the topic."""
        topic = '/action'
        fake_client = _FakeClient()
        active_future = _FakeFuture()
        ProxyActionClient._clients[topic] = {'client': fake_client, 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = active_future
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ACCEPTED

        with patch.object(ProxyActionClient, '_check_topic_available', return_value=True):
            with self.assertRaisesRegex(ProxyAvailabilityError, 'A goal is already active'):
                ProxyActionClient.send_goal(topic, _FakeGoal(), wait_duration=0.01)

        self.assertEqual(len(fake_client.sent_goals), 0)
        self.assertIs(ProxyActionClient._current_goal[topic], active_future)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_ACCEPTED)

    def test_send_goal_rejects_active_goal_before_availability_wait(self):
        """send_goal should short-circuit on active goals before probing server availability."""
        topic = '/action'
        fake_client = _FakeClient()
        ProxyActionClient._clients[topic] = {'client': fake_client, 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = True

        with patch.object(ProxyActionClient, '_check_topic_available') as check_available:
            with self.assertRaisesRegex(ProxyAvailabilityError, 'A goal is already active'):
                ProxyActionClient.send_goal(topic, _FakeGoal(), wait_duration=0.5)

        check_available.assert_not_called()

    def test_send_goal_rolls_back_state_when_async_send_fails(self):
        """send_goal should not leave stale active-goal state if async dispatch fails."""
        topic = '/action'
        fake_client = _FailingClient()
        ProxyActionClient._clients[topic] = {'client': fake_client, 'generation': 1, 'count': 1}
        ProxyActionClient._result[topic] = 'stale-result'
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_SUCCEEDED
        ProxyActionClient._feedback[topic] = 'stale-feedback'
        ProxyActionClient._has_active_goal[topic] = False
        ProxyActionClient._current_goal[topic] = 'stale-goal'

        with patch.object(ProxyActionClient, '_check_topic_available', return_value=True):
            with self.assertRaisesRegex(RuntimeError, 'send failed'):
                ProxyActionClient.send_goal(topic, _FakeGoal(), wait_duration=0.01)

        self.assertEqual(len(fake_client.sent_goals), 1)
        self.assertEqual(ProxyActionClient._result[topic], 'stale-result')
        self.assertEqual(ProxyActionClient._feedback[topic], 'stale-feedback')
        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(ProxyActionClient._has_active_goal[topic])
        self.assertIsNone(ProxyActionClient._current_goal[topic])

    def test_send_goal_rolls_back_state_when_callback_registration_fails(self):
        """send_goal should not leave stale active-goal state if future wiring fails."""
        topic = '/action'
        fake_client = _BrokenCallbackClient()
        ProxyActionClient._clients[topic] = {'client': fake_client, 'generation': 1, 'count': 1}
        ProxyActionClient._result[topic] = 'stale-result'
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_SUCCEEDED
        ProxyActionClient._feedback[topic] = 'stale-feedback'
        ProxyActionClient._has_active_goal[topic] = False
        ProxyActionClient._current_goal[topic] = 'stale-goal'

        with patch.object(ProxyActionClient, '_check_topic_available', return_value=True):
            with self.assertRaisesRegex(RuntimeError, 'cannot register callback'):
                ProxyActionClient.send_goal(topic, _FakeGoal(), wait_duration=0.01)

        self.assertEqual(len(fake_client.sent_goals), 1)
        self.assertEqual(ProxyActionClient._result[topic], 'stale-result')
        self.assertEqual(ProxyActionClient._feedback[topic], 'stale-feedback')
        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(ProxyActionClient._has_active_goal[topic])
        self.assertIsNone(ProxyActionClient._current_goal[topic])

    def test_send_goal_preserves_inline_completion_results(self):
        """Inline completion callbacks should not be overwritten by post-dispatch reset logic."""
        topic = '/action'
        fake_client = _ImmediateDoneClient()
        ProxyActionClient._clients[topic] = {'client': fake_client, 'generation': 1, 'count': 1}

        with patch.object(ProxyActionClient, '_check_topic_available', return_value=True):
            ProxyActionClient.send_goal(topic, _FakeGoal(), wait_duration=0.01)

        self.assertEqual(len(fake_client.sent_goals), 1)
        self.assertEqual(ProxyActionClient._result[topic], 'ok')
        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(ProxyActionClient._has_active_goal[topic])
        self.assertIsNone(ProxyActionClient._current_goal[topic])


if __name__ == '__main__':
    unittest.main()
