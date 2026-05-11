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

"""Regression tests for calculation-state reentry behavior."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

from flexbe_core import EventState
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger

from flexbe_states.calculation_state import CalculationState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState


class TestCalculationStateRegressions(unittest.TestCase):
    """Ensure failed reentry does not reuse stale calculation results."""

    @classmethod
    def setUpClass(cls):
        """Provide minimal logger/state-logger stubs for EventState wrappers."""
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

    @patch('flexbe_states.calculation_state.Logger.logwarn')
    def test_calculation_state_clears_stale_result_after_failure(self, _logwarn):
        """A failed second entry should publish None, not the prior successful result."""
        state = CalculationState(lambda value: value + 1)
        userdata = SimpleNamespace(input_value=2)

        state.on_enter(userdata)
        state._entering = False
        state.execute(userdata)
        self.assertEqual(3, userdata.output_value)

        state._calculation = lambda _value: (_ for _ in ()).throw(RuntimeError('boom'))
        userdata.input_value = 5
        state.on_enter(userdata)
        state._entering = False
        state.execute(userdata)

        self.assertIsNone(userdata.output_value)

    @patch('flexbe_states.flexible_calculation_state.Logger.logwarn')
    def test_flexible_calculation_state_clears_stale_result_after_failure(
        self, _logwarn
    ):
        """A failed second flexible calculation should not leak the prior result."""
        state = FlexibleCalculationState(
            lambda a, b: a - b,
            input_keys=['a', 'b'],
        )
        userdata = {'a': 5, 'b': 2}

        state.on_enter(userdata)
        state._entering = False
        output = SimpleNamespace()
        state.execute(output)
        self.assertEqual(3, output.output_value)

        state._calculation = lambda **_kwargs: (_ for _ in ()).throw(RuntimeError('boom'))
        userdata['a'] = 10
        userdata['b'] = 1
        state.on_enter(userdata)
        state._entering = False
        state.execute(output)

        self.assertIsNone(output.output_value)


if __name__ == '__main__':
    unittest.main()
