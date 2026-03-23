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

"""Regression tests for stale userdata in input-oriented states."""

import pickle
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from action_msgs.msg import GoalStatus

from flexbe_msgs.action import BehaviorInput

from flexbe_states.input_state import InputState
from flexbe_states.selection_state import SelectionState


class _FakeActionClient:

    def __init__(self, send_goal_exception=None, result=None, status=None, active=False):
        self._send_goal_exception = send_goal_exception
        self._result = result
        self._status = status
        self._active = active
        self.removed = 0
        self.sent_goals = []
        self.cancel_calls = 0

    def remove_result(self, _topic):
        self.removed += 1

    def send_goal(self, topic, goal, wait_duration=None):
        self.sent_goals.append((topic, goal, wait_duration))
        if self._send_goal_exception is not None:
            raise self._send_goal_exception

    def has_result(self, _topic):
        return self._result is not None

    def get_result(self, _topic):
        return self._result

    def get_status(self, _topic):
        return self._status

    def is_active(self, _topic):
        return self._active

    def cancel(self, _topic):
        self.cancel_calls += 1


class _SelectionUserdata(dict):

    def __init__(self, items=None, data=None):
        super().__init__()
        if items is not None:
            super().__setitem__('items', items)
            object.__setattr__(self, 'items', items)
        if data is not None:
            super().__setitem__('data', data)

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __setattr__(self, name, value):
        self[name] = value


class _BrokenSelectionUserdata:

    def __init__(self):
        self.assignments = []

    @property
    def data(self):
        return None if not self.assignments else self.assignments[-1]

    @data.setter
    def data(self, value):
        self.assignments.append(value)
        if len(self.assignments) == 1:
            raise RuntimeError('setter boom')


class TestInputStateRegressions(unittest.TestCase):
    """Cover the remaining operator-input branches with direct unit tests."""

    @patch('flexbe_states.input_state.ProxyActionClient')
    def test_input_state_start_and_stop_manage_proxy_client(self, proxy_action_client):
        """Starting and stopping should allocate and remove the action client."""
        proxy_instance = object()
        proxy_action_client.return_value = proxy_instance
        state = object.__new__(InputState)
        state._action_topic = 'flexbe/behavior_input'
        state._client = None

        state.on_start()
        self.assertIs(proxy_instance, state._client)
        proxy_action_client.assert_called_once_with({'flexbe/behavior_input': BehaviorInput}, wait_duration=0.0)

        state.on_stop()
        proxy_action_client.remove_client.assert_called_once_with('flexbe/behavior_input')
        self.assertIsNone(state._client)

    @patch('flexbe_states.input_state.Logger.localinfo')
    def test_input_state_execute_decodes_string_and_pickled_results(self, _localinfo):
        """Execute should decode both string requests and serialized payloads."""
        string_state = object.__new__(InputState)
        string_state._client = _FakeActionClient(result=SimpleNamespace(
            result_code=BehaviorInput.Result.RESULT_OK,
            data='hello world',
        ))
        string_state._action_topic = 'flexbe/behavior_input'
        string_state._request = BehaviorInput.Goal.REQUEST_STRING
        string_state._return = None
        userdata = SimpleNamespace(data=None)

        self.assertEqual('received', string_state.execute(userdata))
        self.assertEqual('hello world', userdata.data)

        payload = {'target': 3}
        binary_state = object.__new__(InputState)
        binary_state._client = _FakeActionClient(result=SimpleNamespace(
            result_code=BehaviorInput.Result.RESULT_OK,
            data=repr(pickle.dumps(payload)),
        ))
        binary_state._action_topic = 'flexbe/behavior_input'
        binary_state._request = 999
        binary_state._return = None
        userdata = SimpleNamespace(data=None)

        self.assertEqual('received', binary_state.execute(userdata))
        self.assertEqual(payload, userdata.data)

    @patch('flexbe_states.input_state.Logger.localinfo')
    @patch('flexbe_states.input_state.Logger.logwarn')
    def test_input_state_execute_handles_bad_data_and_terminal_statuses(self, _logwarn, _localinfo):
        """Execute should report data errors and canceled or aborted action statuses."""
        bad_state = object.__new__(InputState)
        bad_state._client = _FakeActionClient(result=SimpleNamespace(
            result_code=BehaviorInput.Result.RESULT_OK,
            data='not valid literal data',
        ))
        bad_state._action_topic = 'flexbe/behavior_input'
        bad_state._request = 999
        bad_state._return = None
        userdata = SimpleNamespace(data='stale')

        self.assertEqual('data_error', bad_state.execute(userdata))
        self.assertIsNone(userdata.data)

        aborted_state = object.__new__(InputState)
        aborted_state._client = _FakeActionClient(result=SimpleNamespace(
            result_code=BehaviorInput.Result.RESULT_FAILED,
            data='ignored',
        ))
        aborted_state._action_topic = 'flexbe/behavior_input'
        aborted_state._return = None
        userdata = SimpleNamespace(data='stale')
        self.assertEqual('aborted', aborted_state.execute(userdata))
        self.assertIsNone(userdata.data)

        canceled_state = object.__new__(InputState)
        canceled_state._client = _FakeActionClient(result=None, status=GoalStatus.STATUS_CANCELED)
        canceled_state._action_topic = 'flexbe/behavior_input'
        canceled_state._return = None
        self.assertEqual('aborted', canceled_state.execute(SimpleNamespace(data=None)))

        goal_aborted_state = object.__new__(InputState)
        goal_aborted_state._client = _FakeActionClient(result=None, status=GoalStatus.STATUS_ABORTED)
        goal_aborted_state._action_topic = 'flexbe/behavior_input'
        goal_aborted_state._return = None
        self.assertEqual('aborted', goal_aborted_state.execute(SimpleNamespace(data=None)))

    @patch('flexbe_states.input_state.Logger.localinfo')
    @patch('flexbe_states.input_state.Logger.loginfo')
    @patch('flexbe_states.input_state.Logger.loghint')
    def test_input_state_enter_and_exit_manage_goal_and_userdata_shapes(self, _loghint, _loginfo, _localinfo):
        """Entering should send the request and exiting should cancel and clear stale results."""
        state = object.__new__(InputState)
        state._client = _FakeActionClient(active=True, result=SimpleNamespace())
        state._action_topic = 'flexbe/behavior_input'
        state._request = BehaviorInput.Goal.REQUEST_STRING
        state._message = 'Provide value'
        state._timeout = 1.5
        state._return = 'stale'

        userdata = {}
        state.on_enter(userdata)

        self.assertIsNone(userdata['data'])
        self.assertIsNone(state._return)
        self.assertEqual(1, state._client.removed)
        self.assertEqual(1, len(state._client.sent_goals))
        topic, goal, wait_duration = state._client.sent_goals[0]
        self.assertEqual('flexbe/behavior_input', topic)
        self.assertEqual(BehaviorInput.Goal.REQUEST_STRING, goal.request_type)
        self.assertEqual('Provide value', goal.msg)
        self.assertEqual(1.5, wait_duration)

        state.on_exit(userdata)
        self.assertEqual(1, state._client.cancel_calls)
        self.assertEqual(2, state._client.removed)


class TestSelectionStateRegressions(unittest.TestCase):
    """Ensure failed reentry clears stale userdata from prior successful runs."""

    @patch('flexbe_states.input_state.Logger.logwarn')
    @patch('flexbe_states.input_state.Logger.loghint')
    def test_input_state_clears_stale_userdata_before_failed_reentry(
        self, _loghint, _logwarn
    ):
        """A new request should clear prior userdata before reporting no_connection."""
        state = object.__new__(InputState)
        state._client = _FakeActionClient(send_goal_exception=RuntimeError('offline'))
        state._action_topic = 'flexbe/behavior_input'
        state._request = BehaviorInput.Goal.REQUEST_STRING
        state._message = 'Provide value'
        state._timeout = 1.0
        state._return = None
        userdata = SimpleNamespace(data='stale')

        state.on_enter(userdata)

        self.assertIsNone(userdata.data)
        self.assertEqual('no_connection', state._return)
        self.assertEqual(1, state._client.removed)

    @patch('flexbe_states.selection_state.Logger.logwarn')
    @patch('flexbe_states.selection_state.Logger.loghint')
    def test_selection_state_clears_stale_userdata_before_failed_reentry(
        self, _loghint, _logwarn
    ):
        """A new selection request should clear prior userdata before no_connection."""
        state = object.__new__(SelectionState)
        state._client = _FakeActionClient(send_goal_exception=RuntimeError('offline'))
        state._action_topic = 'flexbe/behavior_input'
        state._message = 'Pick one'
        state._timeout = 1.0
        state._return = None
        userdata = {'items': ['left', 'right'], 'data': 'stale'}

        state.on_enter(userdata)

        self.assertIsNone(userdata['data'])
        self.assertEqual('no_connection', state._return)
        self.assertEqual(1, state._client.removed)

    @patch('flexbe_states.selection_state.ProxyActionClient')
    def test_selection_state_start_and_stop_manage_proxy_client(self, proxy_action_client):
        """Starting and stopping should allocate and remove the action client."""
        proxy_instance = object()
        proxy_action_client.return_value = proxy_instance
        state = object.__new__(SelectionState)
        state._action_topic = 'flexbe/behavior_input'
        state._client = None

        state.on_start()
        self.assertIs(proxy_instance, state._client)
        proxy_action_client.assert_called_once_with({'flexbe/behavior_input': BehaviorInput}, wait_duration=0.0)

        state.on_stop()
        proxy_action_client.remove_client.assert_called_once_with('flexbe/behavior_input')
        self.assertIsNone(state._client)

    @patch('flexbe_states.selection_state.Logger.localinfo')
    @patch('flexbe_states.selection_state.Logger.loginfo')
    def test_selection_state_execute_handles_success_error_and_terminal_statuses(self, _loginfo, _localinfo):
        """Execute should cover received, data_error, canceled, and aborted result paths."""
        state = object.__new__(SelectionState)
        state._client = _FakeActionClient(result=SimpleNamespace(
            result_code=BehaviorInput.Result.RESULT_OK,
            data='left',
        ))
        state._action_topic = 'flexbe/behavior_input'
        state._return = None
        userdata = SimpleNamespace(data=None)

        self.assertEqual('received', state.execute(userdata))
        self.assertEqual('left', userdata.data)

        broken_state = object.__new__(SelectionState)
        broken_state._client = _FakeActionClient(result=SimpleNamespace(
            result_code=BehaviorInput.Result.RESULT_OK,
            data='right',
        ))
        broken_state._action_topic = 'flexbe/behavior_input'
        broken_state._return = None
        broken_userdata = _BrokenSelectionUserdata()
        with patch('flexbe_states.selection_state.Logger.logwarn') as logwarn:
            self.assertEqual('data_error', broken_state.execute(broken_userdata))
        logwarn.assert_called_once()
        self.assertIsNone(broken_userdata.data)

        canceled_state = object.__new__(SelectionState)
        canceled_state._client = _FakeActionClient(result=None, status=GoalStatus.STATUS_CANCELED)
        canceled_state._action_topic = 'flexbe/behavior_input'
        canceled_state._return = None
        self.assertEqual('aborted', canceled_state.execute(SimpleNamespace(data=None)))

        aborted_state = object.__new__(SelectionState)
        aborted_state._client = _FakeActionClient(result=None, status=GoalStatus.STATUS_ABORTED)
        aborted_state._action_topic = 'flexbe/behavior_input'
        aborted_state._return = None
        self.assertEqual('aborted', aborted_state.execute(SimpleNamespace(data=None)))

    @patch('flexbe_states.selection_state.Logger.localinfo')
    @patch('flexbe_states.selection_state.Logger.loginfo')
    @patch('flexbe_states.selection_state.Logger.loghint')
    @patch('flexbe_states.selection_state.Logger.localwarn')
    def test_selection_state_enter_and_exit_cover_missing_items_and_cleanup(
        self, _localwarn, _loghint, _loginfo, _localinfo
    ):
        """Entering should validate items, send requests, and exiting should cancel active goals."""
        missing_items_state = object.__new__(SelectionState)
        missing_items_state._client = _FakeActionClient()
        missing_items_state._action_topic = 'flexbe/behavior_input'
        missing_items_state._message = 'Pick one'
        missing_items_state._timeout = 2.0
        missing_items_state._return = None
        missing_items_state._name = 'selection'
        userdata = _SelectionUserdata(data='stale')

        missing_items_state.on_enter(userdata)

        self.assertEqual('aborted', missing_items_state._return)
        self.assertIsNone(userdata['data'])
        self.assertEqual([], missing_items_state._client.sent_goals)

        active_state = object.__new__(SelectionState)
        active_state._client = _FakeActionClient(active=True, result=SimpleNamespace())
        active_state._action_topic = 'flexbe/behavior_input'
        active_state._message = 'Choose path'
        active_state._timeout = 0.5
        active_state._return = 'stale'
        userdata = _SelectionUserdata(items=['left', 'right'], data='stale')

        active_state.on_enter(userdata)

        self.assertIsNone(active_state._return)
        self.assertIsNone(userdata['data'])
        self.assertEqual(1, active_state._client.removed)
        self.assertEqual(1, len(active_state._client.sent_goals))
        topic, goal, wait_duration = active_state._client.sent_goals[0]
        self.assertEqual('flexbe/behavior_input', topic)
        self.assertEqual(BehaviorInput.Goal.REQUEST_SELECTION, goal.request_type)
        self.assertEqual(['left', 'right'], list(goal.items))
        self.assertEqual('Choose path', goal.msg)
        self.assertEqual(0.5, wait_duration)

        active_state.on_exit(userdata)
        self.assertEqual(1, active_state._client.cancel_calls)


if __name__ == '__main__':
    unittest.main()
