#!/usr/bin/env python3

# Copyright 2024 Philipp Schillinger, Team ViGIR, Christopher Newport University
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


"""Defines class for testing FlexBE states."""

import re

from flexbe_core.core.user_data import UserData

from .data_provider import DataProvider
from .logger import Logger
from .test_context import PyTestContext, TestContext
from .test_interface import TestInterface


class Tester:
    """Basic class for testing FlexBE states."""

    __test__ = False  # Do not pytest this class (it is the test!)

    def __init__(self, node, executor=None):
        """Initialize Test instance."""
        self._tests = {}
        self.node = node
        self.executor = executor
        self._import_only = False

        Logger.initialize(node)

    def import_interface(self, name, config):
        """Import interface."""
        try:
            self.node.get_logger().info(f"Importing test configuration for '{name}'\n config: {config} ...")
            self._verify_config(config)
        except Exception as e:
            Logger.print_title(name, 'Invalid', None)
            Logger.print_error('invalid test specification!\n\t%s' % str(e))
            Logger.print_result(name, False)
            self._tests['test_%s_pass' % name] = self._test_config_invalid(str(e))
            return None

        # allow to specify behavior name instead of generated module and class
        if 'name' in config:
            config['path'] += '.%s_sm' % re.sub(r'[^\w]', '_', config['name'].lower())
            config['class'] = '%sSM' % re.sub(r'[^\w]', '', config['name'])

        self._import_only = config.get('import_only', False)
        Logger.print_title(name, config['class'], config['outcome'] if not self._import_only else None)

        # import test subject
        try:
            test_interface = TestInterface(self.node, config['path'], config['class'])
            return test_interface
        except Exception as e:
            Logger.print_failure('unable to import state %s (%s):\n\t%s' %
                                 (config['class'], config['path'], str(e)))
            self._tests['test_%s_pass' % name] = self._test_pass(False)
            return None

    def run_pytest(self, name, config, timeout_sec=None, max_cnt=50):
        """Run pytest."""
        self.node.get_logger().info(f" Running pytest setup for '{name}' with timeout_sec={timeout_sec} max_cnt={max_cnt} ...")
        return self.run_test(name, config, context=PyTestContext(timeout_sec=timeout_sec, max_cnt=max_cnt))

    def run_test(self, name, config, context=None):
        """Run test."""
        test_interface = self.import_interface(name, config)

        if test_interface is None:
            return 0

        if self._import_only:
            success = True
            self.node.get_logger().info(f" Successfully imported for '{name}' = ...")
            Logger.print_result(name, success)
            self._tests['test_%s_pass' % name] = self._test_pass(success)
            return 1 if success else 0

        # load data source
        try:
            self.node.get_logger().info(f" Get data provider for '{name}' ...")
            data = DataProvider(self.node, bagfile=None)
        except Exception as e:
            Logger.print_failure('unable to load data source %s:\n\t%s' %
                                 (config['data'], str(e)))
            self._tests['test_%s_pass' % name] = self._test_pass(False)
            return 0

        # prepare test context
        if context is None:
            # If not a PyTestContext
            context = TestContext()

        # run test context
        with context:
            if not context.verify():
                Logger.print_error('failed to initialize test context:\n\t%s' % config['launch'])
                self._tests['test_%s_pass' % name] = self._test_pass(False)
                return 0

            # instantiate test subject
            self.node.get_logger().info(f" Instantiate '{name}' with params ...")
            params = {key: data.parse(value) for key, value in list(config.get('params', {}).items())}
            try:
                test_interface.instantiate(params)
            except Exception as e:
                Logger.print_failure('unable to instantiate %s (%s) with params:\n\t%s\n\t%s' %
                                     (config['class'], config['path'], str(params), str(e)))
                self._tests['test_%s_pass' % name] = self._test_pass(False)
                return 0

            # prepare user data
            self.node.get_logger().info(f" Prepare userdata for '{name}' ...")
            userdata = UserData()
            for input_key, input_value in list(config.get('input', {}).items()):
                userdata[input_key] = data.parse(input_value)
            expected = {key: data.parse(value) for key, value in config.get('output', {}).items()}

            # run test subject
            try:
                self.node.get_logger().info(f" Execute '{name}' ...")
                if self.executor is not None:
                    def spin_cb():
                        self.executor.spin_once(timeout_sec=0.01)
                else:
                    spin_cb = context.spin_once
                outcome = test_interface.execute(userdata,
                                                 context,
                                                 spin_cb=spin_cb)
            except Exception as exc:
                self.node.get_logger().info(f" failed to execute '{name}' \n  {exc}")
                Logger.print_failure('failed to execute %s (%s)\n\t%s' %
                                     (config['class'], config['path'], str(exc)))
                self._tests['test_%s_pass' % name] = self._test_pass(False)
                return 0

            if config.get('require_launch_success', False):
                self.node.get_logger().info(f" Wait for finishing context for '{name}' ...")
                context.wait_for_finishing()

        # evaluate outcome
        self._tests['test_%s_outcome' % name] = self._test_outcome(outcome, config['outcome'])
        outcome_ok = outcome == config['outcome']
        if outcome_ok:
            Logger.print_positive('correctly returned outcome %s' % outcome)
        else:
            Logger.print_negative('wrong outcome: %s' % outcome)

        # evaluate output
        output_ok = True
        for expected_key, expected_value in list(expected.items()):
            if expected_key in userdata:
                equals = userdata[expected_key] == expected_value
                self._tests['test_%s_output_%s' % (name, expected_key)] = \
                    self._test_output(userdata[expected_key], expected_value)
                if not equals:
                    Logger.print_negative('wrong result for %s: %s != %s' %
                                          (expected_key, userdata[expected_key], expected_value))
                    output_ok = False
            else:
                Logger.print_negative('no result for %s' % expected_key)
                output_ok = False

        if not context.success and config.get('require_launch_success', False):
            Logger.print_negative('Launch file did not exit cleanly')
            output_ok = False

        if len(expected) > 0 and output_ok:
            Logger.print_positive('all result outputs match expected')

        # report result
        success = outcome_ok and output_ok
        # self.node.get_logger().info(f" Success for '{name}' = {success} = {outcome_ok} and {output_ok}...")
        Logger.print_result(name, success)
        self._tests['test_%s_pass' % name] = self._test_pass(success)
        return 1 if success else 0

    def _verify_config(self, config):
        if isinstance(config, Exception):
            raise config
        if not isinstance(config, dict):
            raise AssertionError('config needs to be a dictionary but is:\n\t%s' % str(config))
        assert 'path' in config
        assert 'class' in config or 'name' in config
        assert 'outcome' in config or config.get('import_only', False)

    def _test_output(self, value, expected):
        def _test_call(test_self):
            test_self.assertEqual(value, expected, 'Output value %s does not match expected %s' % (value, expected))
        return _test_call

    def _test_outcome(self, outcome, expected):
        def _test_call(test_self):
            test_self.assertEqual(outcome, expected, 'Outcome %s does not match expected %s' % (outcome, expected))
        return _test_call

    def _test_pass(self, passed):
        def _test_call(test_self):
            test_self.assertTrue(passed, 'Did not pass configured tests.')
        return _test_call

    def _test_config_invalid(self, config):
        def _test_call(test_self):
            test_self.fail('Test config is invalid: %s' % config)
        return _test_call
