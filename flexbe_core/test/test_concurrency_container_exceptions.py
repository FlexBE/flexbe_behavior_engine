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

"""Unit tests for ConcurrencyContainer exception handling paths."""

import unittest
from unittest.mock import patch

from flexbe_core.core.concurrency_container import ConcurrencyContainer
from flexbe_core.core.exceptions import StateError, StateMachineError


class _RaisingState:

    def __init__(self, exc):
        self.name = 'state'
        self.path = '/state'
        self.input_keys = []
        self.output_keys = []
        self._inner_sync_request = False
        self._exc = exc

    def execute(self, _userdata):
        raise self._exc


class TestConcurrencyContainerExceptions(unittest.TestCase):
    """Validate typed and wrapped exception behavior for single-state execution."""

    def setUp(self):
        """Create a minimal container instance with required internals."""
        self.cc = object.__new__(ConcurrencyContainer)
        self.cc._name = 'cc'
        self.cc._userdata = {}
        self.cc._remappings = {'state': {}}
        self.cc._last_exception = None
        self.cc._current_state = None

    def test_execute_single_state_preserves_domain_exception(self):
        """Domain exceptions should be re-raised without type conversion."""
        domain_exc = StateMachineError('domain fail')
        state = _RaisingState(domain_exc)

        with patch('flexbe_core.core.concurrency_container.Logger.logerr'), \
                patch('flexbe_core.core.concurrency_container.Logger.localinfo'):
            with self.assertRaises(StateMachineError) as ctx:
                self.cc._execute_single_state(state)

        self.assertIs(ctx.exception, domain_exc)
        self.assertIs(self.cc._last_exception, domain_exc)

    def test_execute_single_state_wraps_unknown_exception(self):
        """Unknown exceptions should be wrapped as StateError."""
        state = _RaisingState(RuntimeError('unexpected fail'))

        with patch('flexbe_core.core.concurrency_container.Logger.logerr'), \
                patch('flexbe_core.core.concurrency_container.Logger.localinfo'):
            with self.assertRaises(StateError) as ctx:
                self.cc._execute_single_state(state)

        self.assertEqual(str(ctx.exception), 'unexpected fail')
        self.assertIsInstance(self.cc._last_exception, StateError)

    def test_notify_skipped_ignores_missing_current_state(self):
        """Skip notifications should tolerate inactive concurrency containers."""
        self.cc._is_controlled = False
        self.cc._current_state = None

        self.cc._notify_skipped()


if __name__ == '__main__':
    unittest.main()
