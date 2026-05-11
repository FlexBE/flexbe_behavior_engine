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

"""Regression tests for TestInterface state classification."""

import types
import unittest
from builtins import __import__ as builtin_import
from unittest.mock import patch

from flexbe_core import EventState

from flexbe_testing.test_interface import TestInterface


class _FakeLogger:

    def info(self, *_args, **_kwargs):
        pass


class _FakeContext:

    def ok(self):
        return True


class _FakeNode:

    def __init__(self):
        self._logger = _FakeLogger()
        self.context = _FakeContext()

    def get_logger(self):
        return self._logger


class _BrokenState(EventState):

    @classmethod
    def initialize_ros(cls, _node):
        raise RuntimeError('broken ros init')

    def __init__(self):
        super().__init__(outcomes=['done'])


class _FakeBehavior:

    def __init__(self, node=None):
        self.node = node


class TestTestInterface(unittest.TestCase):
    """Ensure TestInterface does not hide state initialization failures."""

    @patch('flexbe_testing.test_interface.Logger.print_positive')
    @patch('flexbe_testing.test_interface.initialize_flexbe_core')
    def test_state_initialize_ros_failure_is_not_reclassified(
        self, _initialize_flexbe_core, _print_positive
    ):
        """Propagate initialize_ros failures from real EventState classes."""
        package = types.SimpleNamespace(
            BrokenState=_BrokenState,
            __name__='fake_pkg.fake_module',
        )
        _BrokenState.__module__ = 'fake_pkg.fake_module'

        with patch('builtins.__import__',
                   side_effect=lambda name, *args, **kwargs:
                   package if name == 'fake_pkg.fake_module'
                   else builtin_import(name, *args, **kwargs)):
            with self.assertRaisesRegex(RuntimeError, 'broken ros init'):
                TestInterface(_FakeNode(), 'fake_pkg.fake_module', 'BrokenState')

    @patch('flexbe_testing.test_interface.Logger.print_positive')
    @patch('flexbe_testing.test_interface.initialize_flexbe_core')
    def test_behavior_class_skips_initialize_ros_call(
        self, _initialize_flexbe_core, _print_positive
    ):
        """Treat non-state classes as behaviors without requiring initialize_ros."""
        package = types.SimpleNamespace(
            FakeBehavior=_FakeBehavior,
            __name__='fake_pkg.fake_behavior',
        )
        _FakeBehavior.__module__ = 'fake_pkg.fake_behavior'

        with patch('builtins.__import__',
                   side_effect=lambda name, *args, **kwargs:
                   package if name == 'fake_pkg.fake_behavior'
                   else builtin_import(name, *args, **kwargs)):
            interface = TestInterface(_FakeNode(), 'fake_pkg.fake_behavior', 'FakeBehavior')

        self.assertFalse(interface.is_state())


if __name__ == '__main__':
    unittest.main()
