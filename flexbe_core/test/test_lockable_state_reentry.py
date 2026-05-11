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

"""Tests that LockableState.on_enter resets _locked and _stored_outcome on re-entry."""

from types import SimpleNamespace
import unittest

from flexbe_core.core.event_state import EventState
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger


class _SimpleLockableState(EventState):

    def __init__(self):
        super().__init__(outcomes=['done', 'failed'])

    def execute(self, userdata):
        return None


class TestLockableStateReentry(unittest.TestCase):
    """Verify that LockableState clears stale lock/outcome state on each entry."""

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

    def test_on_enter_clears_locked(self):
        """on_enter must reset _locked to False regardless of prior value."""
        state = _SimpleLockableState()
        state._locked = True
        state.on_enter({})
        self.assertFalse(state._locked)

    def test_on_enter_clears_stored_outcome(self):
        """on_enter must reset _stored_outcome to None regardless of prior value."""
        state = _SimpleLockableState()
        state._stored_outcome = 'done'
        state.on_enter({})
        self.assertIsNone(state._stored_outcome)

    def test_on_enter_clears_both_simultaneously(self):
        """Both _locked and _stored_outcome must be cleared in a single on_enter call."""
        state = _SimpleLockableState()
        state._locked = True
        state._stored_outcome = 'failed'
        state.on_enter({})
        self.assertFalse(state._locked)
        self.assertIsNone(state._stored_outcome)

    def test_reentry_after_forced_exit_does_not_carry_stale_outcome(self):
        """Re-entered state should not immediately fire an outcome left from a prior visit."""
        state = _SimpleLockableState()
        # Simulate state being forced out while a stored outcome was pending
        state._locked = False
        state._stored_outcome = 'failed'
        # Re-enter: on_enter must clear the stale outcome
        state.on_enter({})
        self.assertIsNone(state._stored_outcome, 'Stale stored_outcome should be cleared on re-entry')

    def test_reentry_after_forced_exit_while_locked(self):
        """Re-entered state should not be locked due to a prior visit's lock state."""
        state = _SimpleLockableState()
        state._locked = True
        state._stored_outcome = 'done'
        state.on_enter({})
        self.assertFalse(state._locked, 'Stale _locked should be cleared on re-entry')
        self.assertIsNone(state._stored_outcome, 'Stale stored_outcome should be cleared on re-entry')


if __name__ == '__main__':
    unittest.main()
