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

"""Unit tests for ProxyActionClient cancel callback failure handling."""

import unittest
from unittest.mock import patch

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

from flexbe_core.proxy.proxy_action_client import ProxyActionClient


class _FailingFuture:

    def result(self):
        raise RuntimeError('cancel failure')


class _FakeCancelResultFuture:

    def __init__(self, return_code):
        self._return_code = return_code

    def result(self):
        return type('CancelResult', (), {'return_code': self._return_code})()


class TestProxyActionClientCancelCallback(unittest.TestCase):
    """Validate cancel callback updates state on future exceptions."""

    def setUp(self):
        """Reset static proxy state before each test."""
        ProxyActionClient._clients.clear()
        ProxyActionClient._result_status.clear()
        ProxyActionClient._has_active_goal.clear()
        ProxyActionClient._current_goal.clear()
        ProxyActionClient._client_generation_counter = 0
        ProxyActionClient._is_shutting_down = False

    def test_cancel_callback_sets_aborted_on_exception(self):
        """Cancel callback should preserve active tracking if cancel completion is unknown."""
        topic = '/action'
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = object()
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ACCEPTED

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localwarn'):
            ProxyActionClient._cancel_callback(_FailingFuture(), topic, 1,
                                               GoalStatus.STATUS_ACCEPTED, True,
                                               ProxyActionClient._current_goal[topic])

        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_ACCEPTED)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertIsNotNone(ProxyActionClient._current_goal[topic])

    def test_cancel_callback_transitions_to_canceled_on_ack_and_preserves_goal_tracking(self):
        """Cancel ACK (ERROR_NONE) should set STATUS_CANCELED; goal tracking stays until terminal result arrives."""
        topic = '/action'
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = True
        current_goal = object()
        ProxyActionClient._current_goal[topic] = current_goal
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_CANCELING

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'):
            ProxyActionClient._cancel_callback(_FakeCancelResultFuture(CancelGoal.Response.ERROR_NONE), topic, 1,
                                               GoalStatus.STATUS_ACCEPTED, True, current_goal)

        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_CANCELED)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertIs(ProxyActionClient._current_goal[topic], current_goal)

    def test_cancel_callback_preserves_active_goal_on_rejected_cancel(self):
        """Cancel callback should preserve active tracking when the action server rejects cancel."""
        topic = '/action'
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = object()
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ACCEPTED

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localwarn'):
            ProxyActionClient._cancel_callback(_FakeCancelResultFuture(CancelGoal.Response.ERROR_REJECTED), topic, 1,
                                               GoalStatus.STATUS_ACCEPTED, True,
                                               ProxyActionClient._current_goal[topic])

        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_ACCEPTED)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertIsNotNone(ProxyActionClient._current_goal[topic])

    def test_cancel_callback_does_not_restore_finished_goal_state(self):
        """Late cancel failure should not resurrect a goal that already reached terminal result state."""
        topic = '/action'
        previous_goal = object()
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = False
        ProxyActionClient._current_goal[topic] = None
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_SUCCEEDED

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localwarn'):
            ProxyActionClient._cancel_callback(_FakeCancelResultFuture(CancelGoal.Response.ERROR_REJECTED), topic, 1,
                                               GoalStatus.STATUS_ACCEPTED, True, previous_goal)

        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_SUCCEEDED)
        self.assertFalse(ProxyActionClient._has_active_goal[topic])
        self.assertIsNone(ProxyActionClient._current_goal[topic])

    def test_cancel_callback_does_not_restore_after_new_goal_replaces_old_one(self):
        """Late cancel failure should not overwrite state for a newer goal on the same topic."""
        topic = '/action'
        previous_goal = object()
        new_goal = object()
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = new_goal
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_UNKNOWN

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localwarn'):
            ProxyActionClient._cancel_callback(_FakeCancelResultFuture(CancelGoal.Response.ERROR_REJECTED), topic, 1,
                                               GoalStatus.STATUS_ACCEPTED, True, previous_goal)

        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_UNKNOWN)
        self.assertTrue(ProxyActionClient._has_active_goal[topic])
        self.assertIs(ProxyActionClient._current_goal[topic], new_goal)

    def test_cancel_callback_does_not_reopen_terminal_state_after_accepted_cancel(self):
        """Late accepted cancel callback should not overwrite a terminal result that already arrived."""
        topic = '/action'
        previous_goal = object()
        ProxyActionClient._clients[topic] = {'client': object(), 'generation': 1, 'count': 1}
        ProxyActionClient._has_active_goal[topic] = False
        ProxyActionClient._current_goal[topic] = None
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_CANCELED

        with patch('flexbe_core.proxy.proxy_action_client.Logger.localinfo'):
            ProxyActionClient._cancel_callback(_FakeCancelResultFuture(CancelGoal.Response.ERROR_NONE), topic, 1,
                                               GoalStatus.STATUS_ACCEPTED, True, previous_goal)

        self.assertEqual(ProxyActionClient._result_status[topic], GoalStatus.STATUS_CANCELED)
        self.assertFalse(ProxyActionClient._has_active_goal[topic])
        self.assertIsNone(ProxyActionClient._current_goal[topic])


if __name__ == '__main__':
    unittest.main()
