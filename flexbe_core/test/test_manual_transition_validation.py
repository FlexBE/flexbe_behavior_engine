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

"""Unit tests for manual transition outcome index validation."""

import unittest
from unittest.mock import patch

from flexbe_core.core.concurrency_container import ConcurrencyContainer
from flexbe_core.core.manually_transitionable_state import ManuallyTransitionableState
from flexbe_core.core.operatable_state_machine import OperatableStateMachine


class _FakeCommand:
    """Minimal command message representation for tests."""

    def __init__(self, target, outcome):
        self.target = target
        self.outcome = outcome


class _FakeSubscriber:
    """Minimal buffered subscriber behavior for transition command tests."""

    def __init__(self, command):
        self._command = command
        self._buffered = True

    def has_buffered(self, _topic):
        """Return True while command remains buffered."""
        return self._buffered

    def peek_at_buffer(self, _topic):
        """Peek at current command message."""
        return self._command

    def peek_if_buffered(self, _topic):
        """Peek at current command message when it remains buffered."""
        return self._command if self._buffered else None

    def get_from_buffer(self, _topic):
        """Pop buffered command message."""
        self._buffered = False
        return self._command


class _FakePublisher:
    """Collect published feedback messages."""

    def __init__(self):
        self.messages = []

    def publish(self, _topic, msg):
        """Record published message."""
        self.messages.append(msg)


class _FakeState:
    """Minimal state for concurrency manual-transition bookkeeping tests."""

    def __init__(self):
        self.name = 'state'
        self.path = '/cc/state'
        self.state_id = 301
        self.outcomes = ['done']
        self.input_keys = []
        self.output_keys = []
        self.target_wakeup_ns = 0
        self._force_transition = False
        self._manual_transition_requested = None
        self._inner_sync_request = False
        self._exited = False
        self._entering = False
        self._last_outcome = None
        self.exit_calls = 0
        self.publish_calls = []

    def on_exit(self, _userdata):
        """Record explicit exit handling."""
        self.exit_calls += 1

    def _publish_outcome(self, outcome):
        """Record published outcome."""
        self.publish_calls.append(outcome)


class TestManualTransitionValidation(unittest.TestCase):
    """Validate invalid manual transition outcomes are rejected safely."""

    def test_state_manual_transition_invalid_outcome_returns_none(self):
        """State-level manual transition should reject out-of-range outcome index."""
        state = object.__new__(ManuallyTransitionableState)
        state._is_controlled = True
        state._state_id = 100
        state._outcomes = ['done']
        state._name = 'state'
        state._parent = None
        state._path = '/state'
        state._sub = _FakeSubscriber(_FakeCommand(target=100, outcome=99))
        state._pub = _FakePublisher()
        state._force_transition = False
        state._manual_transition_requested = None

        with patch('flexbe_core.core.manually_transitionable_state.Logger.localerr'):
            outcome = state._manually_transitionable_execute()

        self.assertIsNone(outcome)
        self.assertFalse(state._force_transition)

    def test_osm_manual_transition_invalid_outcome_returns_none(self):
        """Container-level manual transition should reject out-of-range outcome index."""
        sm = object.__new__(OperatableStateMachine)
        sm._is_controlled = True
        sm._state_id = 200
        sm._outcomes = ['done']
        sm._name = 'sm'
        sm._path = '/sm'
        sm._parent = None
        sm._sub = _FakeSubscriber(_FakeCommand(target=200, outcome=99))
        sm._pub = _FakePublisher()
        sm._manual_transition_requested = None
        sm._last_requested_outcome = None

        with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'), \
                patch('flexbe_core.core.operatable_state_machine.Logger.localerr'):
            outcome = sm._execute_current_state()

        self.assertIsNone(outcome)

    def test_cc_manual_transition_invalid_outcome_returns_none(self):
        """Concurrency container manual transition should reject invalid index safely."""
        cc = object.__new__(ConcurrencyContainer)
        cc._is_controlled = True
        cc._state_id = 300
        cc._outcomes = ['done']
        cc._name = 'cc'
        cc._path = '/cc'
        cc._parent = None
        cc._sub = _FakeSubscriber(_FakeCommand(target=300, outcome=99))
        cc._pub = _FakePublisher()
        cc._inner_sync_request = False
        cc._current_state = None
        cc._manual_transition_requested = None
        cc._last_requested_outcome = None

        with patch('flexbe_core.core.concurrency_container.Logger.localinfo'), \
                patch('flexbe_core.core.concurrency_container.Logger.localerr'):
            outcome = cc._execute_current_state()

        self.assertIsNone(outcome)

    def test_cc_manual_transition_marks_child_exited(self):
        """Concurrency child manual transitions should finalize child lifecycle bookkeeping."""
        cc = object.__new__(ConcurrencyContainer)
        state = _FakeState()
        cc._is_controlled = True
        cc._state_id = 300
        cc._outcomes = ['done']
        cc._name = 'cc'
        cc._path = '/cc'
        cc._parent = None
        cc._sub = _FakeSubscriber(_FakeCommand(target=state.state_id, outcome=0))
        cc._pub = _FakePublisher()
        cc._inner_sync_request = False
        cc._current_state = None
        cc._manual_transition_requested = None
        cc._last_requested_outcome = None
        cc._force_transition = False
        cc._returned_outcomes = {}
        cc._conditions = {}
        cc._states = [state]
        cc._userdata = {}
        cc._remappings = {state.name: {}}

        with patch('flexbe_core.core.concurrency_container.Logger.localinfo'), \
                patch('flexbe_core.core.concurrency_container.Logger.localerr'), \
                patch('flexbe_core.core.concurrency_container.Logger.localwarn'):
            outcome = cc._execute_current_state()

        self.assertIsNone(outcome)
        self.assertEqual(cc._returned_outcomes[state.name], 'done')
        self.assertEqual(state.exit_calls, 1)
        self.assertTrue(state._exited)
        self.assertTrue(state._entering)
        self.assertEqual(state._last_outcome, 'done')
        self.assertEqual(state.publish_calls, ['done'])


if __name__ == '__main__':
    unittest.main()
