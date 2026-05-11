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


"""Helper coverage tests for flexbe_testing data provider and logger."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from flexbe_testing.data_provider import DataProvider
from flexbe_testing.logger import Logger


class _FakeParameter:

    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        return SimpleNamespace(bool_value=self._value)


class _FakeRclpyLogger:

    def __init__(self):
        self.info = lambda *_args, **_kwargs: None
        self.warn = lambda *_args, **_kwargs: None
        self.error = lambda *_args, **_kwargs: None
        self.debug = lambda *_args, **_kwargs: None


class _FakeNode:

    def __init__(self, params=None):
        self._params = dict(params or {})
        self._logger = _FakeRclpyLogger()

    def get_parameter(self, name):
        if name not in self._params:
            raise RuntimeError(f'missing parameter {name}')
        return _FakeParameter(self._params[name])

    def declare_parameter(self, name, value):
        self._params[name] = value
        return _FakeParameter(value)

    def get_logger(self):
        return self._logger


class TestDataProviderLoggerHelpers(unittest.TestCase):
    """Exercise remaining helper branches in data provider and logger."""

    def tearDown(self):
        Logger._node = None
        Logger._counter_value = 0

    def test_data_provider_parse_covers_lambda_none_escaped_and_bag_paths(self):
        """DataProvider.parse should handle callable, None, escaped slash, and bag-backed topic paths."""
        provider = DataProvider(_FakeNode())
        provider._bag = SimpleNamespace(read_next=lambda: ('/topic', {'data': 7}, None))

        self.assertEqual(42, provider.parse('lambda : 42')())
        self.assertIsNone(provider.parse('None'))
        self.assertEqual('/escaped', provider.parse('//escaped'))
        self.assertEqual({'data': 7}, provider.parse('/topic'))

    def test_data_provider_parse_reports_bag_errors_and_eval_failures_as_strings(self):
        """DataProvider.parse should log failures and fall back to strings when parsing fails."""
        provider = DataProvider(_FakeNode())
        provider._bag = SimpleNamespace(read_next=lambda: (_ for _ in ()).throw(RuntimeError('bag boom')))

        with patch('flexbe_testing.data_provider.Logger.print_error') as print_error:
            bag_value = provider.parse('/topic')
            bad_lambda = provider.parse('lambda x: x[')

        self.assertEqual('/topic', bag_value)
        self.assertEqual('lambda x: x[', bad_lambda)
        self.assertEqual(2, print_error.call_count)

    def test_logger_parameter_defaults_and_mute_rclpy_follow_compact_settings(self):
        """Logger should declare missing parameters and mute info or warn channels in compact mode."""
        node = _FakeNode({'~compact_format': True})
        Logger.initialize(node)
        original_error = node.get_logger().error

        self.assertFalse(Logger._param_positive())
        self.assertTrue(Logger._param_negative())
        self.assertTrue(Logger._param_compact())

        Logger.mute_rclpy()

        self.assertIs(node.get_logger().info, node.get_logger().debug)
        self.assertIs(node.get_logger().warn, node.get_logger().debug)
        self.assertIs(node.get_logger().error, original_error)

    def test_logger_print_helpers_emit_expected_lines(self):
        """Logger print helpers should respect parameter toggles and format human-readable output."""
        node = _FakeNode({
            '~compact_format': False,
            '~print_debug_positive': True,
            '~print_debug_negative': True,
            '~mute_info': False,
            '~mute_warn': False,
            '~mute_error': True,
        })
        Logger.initialize(node)

        with patch('builtins.print') as print_mock, \
                patch('traceback.print_exc') as print_exc:
            Logger.print_positive('good')
            Logger.print_negative('bad')
            Logger.print_title('demo', 'DemoCase', result='PASS')
            Logger.print_result('demo', True)
            Logger.print_failure('failure text')
            Logger.print_error('error text')

        print_exc.assert_called_once()
        self.assertEqual(6, print_mock.call_count)
        self.assertIn('good', print_mock.call_args_list[0].args[0])
        self.assertIn('bad', print_mock.call_args_list[1].args[0])
        self.assertIn('DemoCase', print_mock.call_args_list[2].args[0])
        self.assertIn('demo completed', print_mock.call_args_list[3].args[0])
        self.assertIn('failure text', print_mock.call_args_list[4].args[0])
        self.assertIn('error text', print_mock.call_args_list[5].args[0])

    def test_logger_mute_rclpy_and_constructor_cover_remaining_branches(self):
        """Logger should honor explicit mute flags and reject instance construction."""
        node = _FakeNode({
            '~compact_format': False,
            '~mute_info': False,
            '~mute_warn': True,
            '~mute_error': True,
        })
        Logger.initialize(node)
        original_info = node.get_logger().info

        Logger.mute_rclpy()

        self.assertIs(node.get_logger().info, original_info)
        self.assertIs(node.get_logger().warn, node.get_logger().debug)
        self.assertIs(node.get_logger().error, node.get_logger().debug)

        with self.assertRaises(NotImplementedError):
            Logger()


if __name__ == '__main__':
    unittest.main()
