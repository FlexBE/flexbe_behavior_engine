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

"""Regression tests for Tester data provider wiring."""

import unittest
from unittest.mock import patch

from flexbe_testing.tester import Tester


class _FakeLogger:

    def info(self, *_args, **_kwargs):
        pass


class _FakeNode:

    def __init__(self):
        self._logger = _FakeLogger()

    def get_logger(self):
        return self._logger


class _FakeContext:

    success = True

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_val, _exc_tb):
        return False

    def verify(self):
        return True

    def spin_once(self):
        pass

    def wait_for_finishing(self):
        pass


class _FakeTestInterface:

    def __init__(self, execute_result='done', instantiate_exception=None):
        self._execute_result = execute_result
        self._instantiate_exception = instantiate_exception

    def instantiate(self, _params):
        if self._instantiate_exception is not None:
            raise self._instantiate_exception

    def execute(self, _userdata, _context, spin_cb=None):
        if spin_cb is not None:
            spin_cb()
        return self._execute_result


class TestTesterDataProvider(unittest.TestCase):
    """Ensure Tester forwards configured bagfile paths to DataProvider."""

    def _make_tester(self):
        return Tester(_FakeNode(), executor=None, execute_wait=0.0)

    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.DataProvider')
    def test_run_test_passes_configured_bagfile(
        self, data_provider_cls, *_mocks
    ):
        """Forward config['data'] to DataProvider."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
            'data': 'demo_pkg/testdata.bag',
        }
        data_provider = data_provider_cls.return_value
        data_provider.parse.side_effect = lambda value: value

        with patch.object(Tester, 'import_interface', return_value=_FakeTestInterface()):
            success = tester.run_test('demo', config, context=_FakeContext())

        self.assertEqual(1, success)
        data_provider_cls.assert_called_once_with(
            tester.node, bagfile='demo_pkg/testdata.bag'
        )

    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.DataProvider')
    def test_run_test_uses_none_when_data_is_absent(
        self, data_provider_cls, *_mocks
    ):
        """Keep the existing behavior for tests without a bagfile."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
        }
        data_provider = data_provider_cls.return_value
        data_provider.parse.side_effect = lambda value: value

        with patch.object(Tester, 'import_interface', return_value=_FakeTestInterface()):
            success = tester.run_test('demo', config, context=_FakeContext())

        self.assertEqual(1, success)
        data_provider_cls.assert_called_once_with(tester.node, bagfile=None)

    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.Logger.print_failure')
    @patch('flexbe_testing.tester.DataProvider', side_effect=RuntimeError('boom'))
    def test_run_test_reports_provider_failure_without_data_key(
        self, _data_provider_cls, print_failure, *_mocks
    ):
        """Report provider setup failures even when config omits data."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
        }

        with patch.object(Tester, 'import_interface', return_value=_FakeTestInterface()):
            success = tester.run_test('demo', config, context=_FakeContext())

        self.assertEqual(0, success)
        print_failure.assert_called_once_with(
            'unable to load data source None:\n\tboom'
        )

    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.LaunchContext')
    @patch('flexbe_testing.tester.DataProvider')
    def test_run_test_uses_launch_context_for_launch_configs(
        self, data_provider_cls, launch_context_cls, *_mocks
    ):
        """Select LaunchContext when a .test config contains launch data."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
            'launch': '<launch/>',
            'wait_cond': 'ready',
        }
        data_provider = data_provider_cls.return_value
        data_provider.parse.side_effect = lambda value: value
        launch_context = _FakeContext()
        launch_context_cls.return_value = launch_context

        with patch.object(Tester, 'import_interface', return_value=_FakeTestInterface()):
            success = tester.run_test('demo', config)

        self.assertEqual(1, success)
        launch_context_cls.assert_called_once_with(
            tester.node, '<launch/>', wait_cond='ready', execute_wait=0.0
        )

    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.TestContext')
    @patch('flexbe_testing.tester.DataProvider')
    def test_run_test_uses_default_test_context_without_launch(
        self, data_provider_cls, test_context_cls, *_mocks
    ):
        """Keep using the default TestContext when no launch config is present."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
        }
        data_provider = data_provider_cls.return_value
        data_provider.parse.side_effect = lambda value: value
        test_context = _FakeContext()
        test_context_cls.return_value = test_context

        with patch.object(Tester, 'import_interface', return_value=_FakeTestInterface()):
            success = tester.run_test('demo', config)

        self.assertEqual(1, success)
        test_context_cls.assert_called_once_with(tester.node, 0.0)

    @patch('flexbe_testing.tester.LaunchPyTestContext')
    def test_run_pytest_uses_launch_pytest_context_for_launch_configs(
        self, launch_pytest_context_cls
    ):
        """Use a launch-capable pytest context when a pytest config contains launch data."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
            'launch': '<launch/>',
            'wait_cond': 'ready',
        }
        launch_context = object()
        launch_pytest_context_cls.return_value = launch_context

        with patch.object(Tester, 'run_test', return_value=1) as run_test:
            success = tester.run_pytest('demo', config, timeout_sec=1.5, max_cnt=7, execute_wait=0.25)

        self.assertEqual(1, success)
        launch_pytest_context_cls.assert_called_once_with(
            tester.node,
            '<launch/>',
            wait_cond='ready',
            timeout_sec=1.5,
            max_cnt=7,
            execute_wait=0.25,
        )
        run_test.assert_called_once_with('demo', config, context=launch_context)

    @patch('flexbe_testing.tester.PyTestContext')
    def test_run_pytest_uses_plain_pytest_context_without_launch(
        self, pytest_context_cls
    ):
        """Keep using the plain pytest context when no launch config is present."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
        }
        pytest_context = object()
        pytest_context_cls.return_value = pytest_context

        with patch.object(Tester, 'run_test', return_value=1) as run_test:
            success = tester.run_pytest('demo', config, timeout_sec=1.5, max_cnt=7, execute_wait=0.25)

        self.assertEqual(1, success)
        pytest_context_cls.assert_called_once_with(
            tester.node,
            timeout_sec=1.5,
            max_cnt=7,
            execute_wait=0.25,
        )
        run_test.assert_called_once_with('demo', config, context=pytest_context)

    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.TestInterface')
    def test_import_interface_does_not_mutate_name_based_config(
        self, test_interface_cls, _print_title
    ):
        """Derive path/class from name without altering the caller's config."""
        tester = self._make_tester()
        config = {
            'path': 'demo_pkg.behaviors',
            'name': 'Demo Behavior',
            'outcome': 'finished',
        }
        original = dict(config)
        test_interface = object()
        test_interface_cls.return_value = test_interface

        result = tester.import_interface('demo', config)

        self.assertIs(test_interface, result)
        self.assertEqual(original, config)
        test_interface_cls.assert_called_once_with(
            tester.node,
            'demo_pkg.behaviors.demo_behavior_sm',
            'DemoBehaviorSM',
        )

    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.DataProvider')
    @patch('flexbe_testing.tester.TestInterface')
    def test_run_test_name_based_config_imports_single_generated_module(
        self, test_interface_cls, data_provider_cls, *_mocks
    ):
        """Avoid appending the generated behavior module suffix more than once."""
        tester = self._make_tester()
        config = {
            'path': 'demo_pkg.behaviors',
            'name': 'Demo Behavior',
            'outcome': 'done',
        }
        data_provider = data_provider_cls.return_value
        data_provider.parse.side_effect = lambda value: value
        interface = _FakeTestInterface(execute_result='done')
        test_interface_cls.return_value = interface

        success = tester.run_test('demo', config, context=_FakeContext())

        self.assertEqual(1, success)
        test_interface_cls.assert_called_once_with(
            tester.node,
            'demo_pkg.behaviors.demo_behavior_sm',
            'DemoBehaviorSM',
        )

    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.Logger.print_failure')
    @patch('flexbe_testing.tester.DataProvider')
    def test_run_test_name_based_config_uses_derived_class_and_path(
        self, data_provider_cls, print_failure, *_mocks
    ):
        """Use normalized path/class in later failure reporting for name-based configs."""
        tester = self._make_tester()
        config = {
            'path': 'demo_pkg.behaviors',
            'name': 'Demo Behavior',
            'outcome': 'finished',
        }
        data_provider = data_provider_cls.return_value
        data_provider.parse.side_effect = lambda value: value
        interface = _FakeTestInterface(instantiate_exception=RuntimeError('bad params'))

        with patch.object(Tester, 'import_interface', return_value=interface):
            success = tester.run_test('demo', config, context=_FakeContext())

        self.assertEqual(0, success)
        print_failure.assert_called_once_with(
            'unable to instantiate DemoBehaviorSM (demo_pkg.behaviors.demo_behavior_sm) '
            'with params:\n\t{}\n\tbad params'
        )

    @patch('flexbe_testing.tester.Logger.print_error')
    @patch('flexbe_testing.tester.Logger.print_positive')
    @patch('flexbe_testing.tester.Logger.print_negative')
    @patch('flexbe_testing.tester.Logger.print_result')
    @patch('flexbe_testing.tester.Logger.print_title')
    @patch('flexbe_testing.tester.DataProvider')
    def test_run_test_reports_verify_failure_without_launch_key(
        self, data_provider_cls, _print_title, _print_result,
        _print_negative, _print_positive, print_error
    ):
        """Report context verification failures even for non-launch configs."""
        tester = self._make_tester()
        config = {
            'path': 'pkg.module',
            'class': 'DemoState',
            'outcome': 'done',
        }
        data_provider = data_provider_cls.return_value
        data_provider.parse.side_effect = lambda value: value
        context = _FakeContext()
        context.verify = lambda: False

        with patch.object(Tester, 'import_interface', return_value=_FakeTestInterface()):
            success = tester.run_test('demo', config, context=context)

        self.assertEqual(0, success)
        print_error.assert_called_once_with(
            'failed to initialize test context:\n\tNone'
        )


if __name__ == '__main__':
    unittest.main()
