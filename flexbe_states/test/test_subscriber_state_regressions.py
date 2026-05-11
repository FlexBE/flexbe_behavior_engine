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

"""Regression tests for SubscriberState non-blocking behavior."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

from flexbe_states.subscriber_state import SubscriberState


class _FakeSubscriber:

    def __init__(self, has_message=False, message=None):
        self._has_message = has_message
        self._message = message
        self.removed = 0

    def has_msg(self, _topic):
        return self._has_message

    def get_last_msg(self, _topic):
        return self._message

    def remove_last_msg(self, _topic):
        self.removed += 1


class TestSubscriberStateRegressions(unittest.TestCase):
    """Ensure SubscriberState handles non-blocking mode safely."""

    def test_non_blocking_without_message_returns_received_with_none(self):
        """Do not call cached-message accessors when no message is available."""
        state = object.__new__(SubscriberState)
        state._connected = True
        state._blocking = False
        state._topic = '/test'
        state._sub = _FakeSubscriber(has_message=False)
        userdata = SimpleNamespace(message='stale')

        outcome = state.execute(userdata)

        self.assertEqual('received', outcome)
        self.assertIsNone(userdata.message)
        self.assertEqual(0, state._sub.removed)

    def test_non_blocking_with_message_still_consumes_cached_message(self):
        """Keep the existing behavior when a message is actually available."""
        message = SimpleNamespace(data='payload')
        state = object.__new__(SubscriberState)
        state._connected = True
        state._blocking = False
        state._topic = '/test'
        state._sub = _FakeSubscriber(has_message=True, message=message)
        userdata = SimpleNamespace(message=None)

        outcome = state.execute(userdata)

        self.assertEqual('received', outcome)
        self.assertIs(message, userdata.message)
        self.assertEqual(1, state._sub.removed)

    def test_on_stop_unsubscribes_using_state_instance_id(self):
        """Use the original subscription instance id when cleaning up."""
        state = object.__new__(SubscriberState)
        state._connected = True
        state._topic = '/test'

        with patch('flexbe_states.subscriber_state.ProxySubscriberCached.unsubscribe_topic') as unsubscribe:
            state.on_stop()

        unsubscribe.assert_called_once_with('/test', inst_id=id(state))
        self.assertFalse(state._connected)


if __name__ == '__main__':
    unittest.main()
