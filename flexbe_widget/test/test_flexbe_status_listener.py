#!/usr/bin/env python3

# Copyright 2026  Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
##
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


"""Focused tests for the FlexBE status listener utility."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

from flexbe_widget.flexbe_status_listener import flexbe_status_listener_main
from flexbe_widget.flexbe_status_listener import FlexbeStatusListener


class TestFlexbeStatusListener(unittest.TestCase):
    """Exercise status-listener helpers without constructing a ROS node."""

    def _make_listener(self):
        messages = []
        listener = object.__new__(FlexbeStatusListener)
        listener._state_map = None
        listener.get_logger = lambda: SimpleNamespace(info=lambda msg: messages.append(msg))
        return listener, messages

    def test_state_map_callback_records_paths_and_logs_entries(self):
        """State map updates should store the mapping and log each added state path."""
        listener, messages = self._make_listener()

        listener._state_map_callback(SimpleNamespace(
            behavior_id=17,
            state_ids=[101, 102],
            state_paths=['/root/a', '/root/b'],
        ))

        self.assertEqual({101: '/root/a', 102: '/root/b'}, listener._state_map)
        self.assertTrue(any('New state map received for 17' in msg for msg in messages))
        self.assertTrue(any("adding         101 at '/root/a'" in msg for msg in messages))

    def test_callbacks_log_raw_messages_without_state_map(self):
        """Callbacks should echo raw message content before any state map has been received."""
        listener, messages = self._make_listener()

        listener._heartbeat_callback(SimpleNamespace(behavior_id=9, current_state_checksums=[]))
        listener._sync_callback(SimpleNamespace(behavior_id=8, current_state_checksums=[]))
        listener._outcome_callback(SimpleNamespace(data=77))

        self.assertTrue(any('Onboard heartbeat' in msg for msg in messages))
        self.assertTrue(any('Synchronize mirror' in msg for msg in messages))
        self.assertTrue(any('Outcome msg hash value' in msg for msg in messages))

    def test_callbacks_render_known_and_unknown_paths_from_state_ids(self):
        """Callbacks should resolve hashed state ids through the most recent state map."""
        listener, messages = self._make_listener()
        listener._state_map = {11: '/known/path'}

        with patch('flexbe_widget.flexbe_status_listener.StateMap.unhash',
                   side_effect=[(11, 0), (99, 2), (11, 1), (11, 3)]):
            listener._heartbeat_callback(SimpleNamespace(behavior_id=41, current_state_checksums=[1001, 1002]))
            listener._sync_callback(SimpleNamespace(behavior_id=42, current_state_checksums=[1003]))
            listener._outcome_callback(SimpleNamespace(data=1004))

        self.assertTrue(any("['/known/path', 'unknown']" in msg for msg in messages))
        self.assertTrue(any('Synchronize mirror         42' in msg for msg in messages))
        self.assertTrue(any("Outcome  3 from          11 '/known/path'" in msg for msg in messages))

    def test_status_listener_main_initializes_spins_and_shuts_down(self):
        """Main entrypoint should initialize ROS, spin the node, then cleanly destroy and shut down."""
        destroyed = []
        listener = SimpleNamespace(destroy_node=lambda: destroyed.append(True))

        with patch('flexbe_widget.flexbe_status_listener.rclpy.init') as rclpy_init, \
                patch('flexbe_widget.flexbe_status_listener.rclpy.spin') as rclpy_spin, \
                patch('flexbe_widget.flexbe_status_listener.rclpy.shutdown') as rclpy_shutdown, \
                patch('flexbe_widget.flexbe_status_listener.FlexbeStatusListener', return_value=listener), \
                patch('builtins.print') as print_mock:
            flexbe_status_listener_main(args=['--demo'])

        rclpy_init.assert_called_once_with(args=['--demo'])
        rclpy_spin.assert_called_once_with(listener)
        rclpy_shutdown.assert_called_once()
        self.assertEqual([True], destroyed)
        self.assertEqual(2, print_mock.call_count)


if __name__ == '__main__':
    unittest.main()
