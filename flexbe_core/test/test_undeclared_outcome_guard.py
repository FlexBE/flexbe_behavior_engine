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

"""Tests that OperatableState logs and ignores undeclared outcomes instead of crashing."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from flexbe_core.core.event_state import EventState
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger


class _UndeclaredOutcomeState(EventState):
    """State whose execute() returns an outcome not listed in its outcomes."""

    def __init__(self):
        super().__init__(outcomes=['done'])

    def execute(self, userdata):
        return 'not_declared'


class TestUndeclaredOutcomeGuard(unittest.TestCase):
    """Verify that returning an undeclared outcome is caught gracefully in controlled mode."""

    @classmethod
    def setUpClass(cls):
        """Set up fake ROS node infrastructure for all tests."""
        fake_now = SimpleNamespace(nanoseconds=0)
        fake_clock = SimpleNamespace(now=lambda: fake_now)
        fake_logger = SimpleNamespace(
            name='fake_ros_logger',
            info=lambda *args, **kwargs: None,
            debug=lambda *args, **kwargs: None,
            warn=lambda *args, **kwargs: None,
            error=lambda *args, **kwargs: None,
        )
        fake_publisher = SimpleNamespace(publish=lambda *args, **kwargs: None)
        fake_node = SimpleNamespace(
            get_clock=lambda: fake_clock,
            get_logger=lambda: fake_logger,
            create_publisher=lambda *args, **kwargs: fake_publisher,
            destroy_publisher=lambda *args, **kwargs: None,
            has_parameter=lambda *args, **kwargs: False,
            declare_parameter=lambda *args, **kwargs: None,
            get_parameter=lambda *args, **kwargs: SimpleNamespace(
                get_parameter_value=lambda: SimpleNamespace(bool_value=False)
            ),
        )
        Logger._node = fake_node
        Logger._ros_logger = fake_logger
        Logger._pub = fake_publisher
        Logger._last_logged = {}
        Logger._local_info_enabled = True
        Logger._local_warn_enabled = True
        Logger._local_hint_enabled = True
        Logger._local_error_enabled = True
        Logger._local_debug_enabled = False
        StateLogger._node = fake_node
        EventState._node = fake_node

    def _make_controlled_state(self):
        """Return a state configured for controlled (OCS-connected) execution."""
        state = _UndeclaredOutcomeState()
        state._is_controlled = True
        state._force_transition = False
        state._last_requested_outcome = None
        # Fake parent that never allows autonomous transitions
        state._parent = SimpleNamespace(
            name='parent',
            path='',
            is_transition_allowed=lambda name, outcome: False,
            transition_allowed=lambda name, outcome: True,
            get_required_autonomy=lambda outcome, s: 3,
            autonomy_level=3,
        )
        return state

    @patch('flexbe_core.core.operatable_state.Logger.localerr')
    def test_undeclared_outcome_is_logged(self, mock_localerr):
        """An undeclared outcome must be logged as an error in controlled mode."""
        state = self._make_controlled_state()
        state.on_enter({})
        state._entering = False
        state.execute({})
        mock_localerr.assert_called_once()
        self.assertIn('not_declared', str(mock_localerr.call_args))

    @patch('flexbe_core.core.operatable_state.Logger.localerr')
    def test_undeclared_outcome_returns_none(self, _mock_localerr):
        """An undeclared outcome must be suppressed; execute must return None."""
        state = self._make_controlled_state()
        state.on_enter({})
        state._entering = False
        result = state.execute({})
        self.assertIsNone(result)

    @patch('flexbe_core.core.operatable_state.Logger.localerr')
    def test_undeclared_outcome_does_not_raise(self, _mock_localerr):
        """An undeclared outcome must not raise ValueError or any other exception."""
        state = self._make_controlled_state()
        state.on_enter({})
        state._entering = False
        try:
            state.execute({})
        except Exception as exc:
            self.fail(f'execute() raised unexpectedly: {exc}')


if __name__ == '__main__':
    unittest.main()
