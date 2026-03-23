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

"""Tests that RosState.set_rate and set_default_rate reject non-positive rates."""

import unittest

from flexbe_core.core.ros_state import RosState


class TestRateValidation(unittest.TestCase):
    """Verify that zero and negative rates are rejected with ValueError."""

    def setUp(self):
        """Preserve the class-level default rate so tests don't interfere with each other."""
        self._original_default_rate = RosState._default_rate_hz

    def tearDown(self):
        """Restore original default rate."""
        RosState._default_rate_hz = self._original_default_rate

    # --- set_rate (instance method) ---

    def test_set_rate_zero_raises(self):
        """set_rate(0) must raise ValueError."""
        state = RosState.__new__(RosState)
        with self.assertRaises(ValueError):
            state.set_rate(0)

    def test_set_rate_zero_float_raises(self):
        """set_rate(0.0) must raise ValueError."""
        state = RosState.__new__(RosState)
        with self.assertRaises(ValueError):
            state.set_rate(0.0)

    def test_set_rate_negative_raises(self):
        """set_rate(-1) must raise ValueError."""
        state = RosState.__new__(RosState)
        with self.assertRaises(ValueError):
            state.set_rate(-1)

    def test_set_rate_negative_float_raises(self):
        """set_rate(-0.001) must raise ValueError."""
        state = RosState.__new__(RosState)
        with self.assertRaises(ValueError):
            state.set_rate(-0.001)

    def test_set_rate_positive_does_not_raise(self):
        """set_rate with a positive value must not raise."""
        state = RosState.__new__(RosState)
        try:
            state.set_rate(10.0)
        except ValueError:
            self.fail('set_rate(10.0) raised ValueError unexpectedly')

    # --- set_default_rate (class method) ---

    def test_set_default_rate_zero_raises(self):
        """set_default_rate(0) must raise ValueError."""
        with self.assertRaises(ValueError):
            RosState.set_default_rate(0)

    def test_set_default_rate_negative_raises(self):
        """set_default_rate with negative value must raise ValueError."""
        with self.assertRaises(ValueError):
            RosState.set_default_rate(-5.0)

    def test_set_default_rate_zero_float_raises(self):
        """set_default_rate(0.0) must raise ValueError."""
        with self.assertRaises(ValueError):
            RosState.set_default_rate(0.0)


if __name__ == '__main__':
    unittest.main()
