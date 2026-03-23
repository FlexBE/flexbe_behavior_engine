#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
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

"""Unit tests for ProxyTransformListener shutdown behavior."""

import unittest
from unittest.mock import patch

from flexbe_core.proxy import proxy_transform_listener as proxy_transform_listener_module
from flexbe_core.proxy.proxy_transform_listener import ProxyTransformListener


class _FakeListener:
    """Minimal listener exposing unregister()."""

    def __init__(self):
        self.unregister_calls = 0

    def unregister(self):
        """Record explicit shutdown."""
        self.unregister_calls += 1


class TestProxyTransformListener(unittest.TestCase):
    """Validate transform listener teardown semantics."""

    def setUp(self):
        """Reset static proxy state before each test."""
        ProxyTransformListener._listener = None
        ProxyTransformListener._buffer = None
        ProxyTransformListener._node = None

    def test_shutdown_unregisters_listener_and_clears_state(self):
        """Shutdown should explicitly unregister subscriptions before clearing state."""
        listener = _FakeListener()
        buffer = object()
        ProxyTransformListener._listener = listener
        ProxyTransformListener._buffer = buffer

        with patch('builtins.print'):
            ProxyTransformListener.shutdown()

        self.assertEqual(listener.unregister_calls, 1)
        self.assertIsNone(ProxyTransformListener._listener)
        self.assertIsNone(ProxyTransformListener._buffer)

    def test_constructor_reuses_existing_listener_instance(self):
        """Constructing twice should reuse the singleton listener instead of recreating it."""
        created_listeners = []

        class _ConstructedListener(_FakeListener):
            pass

        def _make_listener(buffer, node):
            listener = _ConstructedListener()
            created_listeners.append((buffer, node, listener))
            return listener

        ProxyTransformListener._node = object()
        with patch.object(proxy_transform_listener_module.tf2_ros, 'Buffer', side_effect=[object()]), \
                patch.object(proxy_transform_listener_module.tf2_ros, 'TransformListener', side_effect=_make_listener):
            first = ProxyTransformListener()
            second = ProxyTransformListener()

        self.assertIs(first.listener, second.listener)
        self.assertEqual(len(created_listeners), 1)

    def test_constructor_creates_fresh_listener_after_shutdown(self):
        """A new constructor call after shutdown should create a fresh listener instance."""
        created_listeners = []

        class _ConstructedListener(_FakeListener):
            pass

        def _make_listener(buffer, node):
            listener = _ConstructedListener()
            created_listeners.append(listener)
            return listener

        ProxyTransformListener._node = object()
        with patch.object(proxy_transform_listener_module.tf2_ros, 'Buffer', side_effect=[object(), object()]), \
                patch.object(proxy_transform_listener_module.tf2_ros, 'TransformListener', side_effect=_make_listener), \
                patch('builtins.print'):
            first = ProxyTransformListener()
            first_listener = first.listener
            ProxyTransformListener.shutdown()
            second = ProxyTransformListener()

        self.assertIsNot(first_listener, second.listener)
        self.assertEqual(len(created_listeners), 2)


if __name__ == '__main__':
    unittest.main()
