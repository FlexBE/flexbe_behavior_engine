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

"""Regression tests for LogKeyState format error handling."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from flexbe_core import EventState
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger

from flexbe_states.log_key_state import LogKeyState


class TestLogKeyStateRegressions(unittest.TestCase):
    """Verify that LogKeyState handles format errors gracefully without crashing."""

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

    @patch('flexbe_states.log_key_state.Logger.logwarn')
    def test_bad_format_string_logs_warning_not_crash(self, mock_logwarn):
        """A format string with mismatched placeholders should log a warning, not raise."""
        state = LogKeyState('{} {} too many placeholders')
        userdata = SimpleNamespace(data='only_one_value')
        try:
            state.on_enter(userdata)
        except Exception as exc:
            self.fail(f'on_enter raised unexpectedly: {exc}')
        mock_logwarn.assert_called_once()
        self.assertIn('too many placeholders', str(mock_logwarn.call_args))

    @patch('flexbe_states.log_key_state.Logger.logwarn')
    def test_missing_data_attribute_logs_warning_not_crash(self, mock_logwarn):
        """Missing userdata.data attribute should log a warning, not raise AttributeError."""
        state = LogKeyState('Value: {}')
        userdata = SimpleNamespace()  # no 'data' attribute
        try:
            state.on_enter(userdata)
        except Exception as exc:
            self.fail(f'on_enter raised unexpectedly: {exc}')
        mock_logwarn.assert_called_once()

    @patch('flexbe_states.log_key_state.Logger.logwarn')
    def test_incompatible_format_type_logs_warning_not_crash(self, mock_logwarn):
        """A numeric format applied to a string value should log a warning, not raise TypeError."""
        state = LogKeyState('Number: {:.2f}')
        userdata = SimpleNamespace(data='not_a_number')
        try:
            state.on_enter(userdata)
        except Exception as exc:
            self.fail(f'on_enter raised unexpectedly: {exc}')
        mock_logwarn.assert_called_once()

    @patch('flexbe_states.log_key_state.Logger.log')
    def test_valid_format_string_logs_successfully(self, mock_log):
        """A well-formed format string with matching data should call Logger.log, not logwarn."""
        state = LogKeyState('Counter: {}')
        userdata = SimpleNamespace(data=42)
        with patch('flexbe_states.log_key_state.Logger.logwarn') as mock_warn:
            state.on_enter(userdata)
            mock_warn.assert_not_called()
        mock_log.assert_called_once()
        self.assertIn('42', str(mock_log.call_args))


if __name__ == '__main__':
    unittest.main()
