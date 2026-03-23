#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
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

"""Unit tests for ProxySubscriberCached registration races."""

import threading
import unittest
from unittest.mock import patch

from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached


class _FakeMsg:
    """Minimal message object for subscriber tests."""

    def __init__(self, data):
        self.data = data


class _FakeSubscription:
    """Subscription handle exposing msg_type like rclpy subscriptions do."""

    def __init__(self, msg_type):
        self.msg_type = msg_type


class _FakeExecutor:
    """Run scheduled tasks immediately for deterministic tests."""

    def create_task(self, callback, *args):
        callback(*args)


class _ImmediateMessageNode:
    """Create subscriptions that immediately deliver one message."""

    def __init__(self, message):
        self._message = message
        self.executor = _FakeExecutor()

    def create_subscription(self, msg_type, _topic, callback, _qos):
        callback(self._message)
        return _FakeSubscription(msg_type)


class _BlockingSubscriptionNode:
    """Block subscription creation to expose concurrent subscribe races."""

    def __init__(self):
        self.executor = _FakeExecutor()
        self.started = threading.Event()
        self.allow_finish = threading.Event()
        self.create_calls = 0

    def create_subscription(self, msg_type, _topic, _callback, _qos):
        self.create_calls += 1
        self.started.set()
        self.allow_finish.wait(timeout=1.0)
        return _FakeSubscription(msg_type)


class _FailingSubscriptionNode(_BlockingSubscriptionNode):
    """Block, then fail subscription creation to expose concurrent error propagation."""

    def create_subscription(self, msg_type, _topic, _callback, _qos):
        self.create_calls += 1
        self.started.set()
        self.allow_finish.wait(timeout=1.0)
        raise RuntimeError('create failed')


class _HookedLock:
    """Context manager that runs a hook before entering the protected section."""

    def __init__(self, hook):
        self._hook = hook

    def __enter__(self):
        self._hook()
        return self

    def __exit__(self, exc_type, exc, tb):
        return False


class TestProxySubscriberCached(unittest.TestCase):
    """Validate subscriber registration before first-message delivery."""

    def setUp(self):
        """Reset proxy singleton state before each test."""
        ProxySubscriberCached._topics.clear()
        ProxySubscriberCached._persistant_topics.clear()
        ProxySubscriberCached._node = None

    def test_subscribe_caches_message_delivered_during_subscription_creation(self):
        """The first message should not be dropped during create_subscription()."""
        ProxySubscriberCached._node = _ImmediateMessageNode(_FakeMsg('first'))

        proxy = ProxySubscriberCached()
        with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localinfo'):
            proxy.subscribe('/topic', _FakeMsg, inst_id=1)

        self.assertTrue(proxy.has_msg('/topic'))
        self.assertEqual(proxy.get_last_msg('/topic').data, 'first')

    def test_subscribe_invokes_initial_callback_for_message_delivered_immediately(self):
        """The initial callback should already be active for the first message."""
        ProxySubscriberCached._node = _ImmediateMessageNode(_FakeMsg('first'))
        received = []

        def _record(msg):
            received.append(msg.data)

        proxy = ProxySubscriberCached()
        with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localinfo'):
            proxy.subscribe('/topic', _FakeMsg, callback=_record, inst_id=2)

        self.assertEqual(received, ['first'])

    def test_peek_if_buffered_returns_none_without_messages(self):
        """peek_if_buffered() should avoid a second buffer-state check when empty."""
        ProxySubscriberCached._topics['/topic'] = {
            'subscription': _FakeSubscription(_FakeMsg),
            'last_msg': None,
            'buffered': True,
            'msg_queue': [],
            'callbacks': {},
            'callback_items': (),
            'subscribers': [1],
        }

        self.assertIsNone(ProxySubscriberCached.peek_if_buffered('/topic'))

    def test_concurrent_subscribe_waits_for_inflight_subscription_creation(self):
        """Concurrent subscribe calls on the same topic should not dereference a None subscription."""
        blocking_node = _BlockingSubscriptionNode()
        ProxySubscriberCached._node = blocking_node
        proxy = ProxySubscriberCached()
        errors = []

        def _subscribe(inst_id):
            try:
                with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localinfo'):
                    proxy.subscribe('/topic', _FakeMsg, inst_id=inst_id)
            except Exception as exc:  # pylint: disable=W0703
                errors.append(exc)

        first = threading.Thread(target=_subscribe, args=(1,))
        second = threading.Thread(target=_subscribe, args=(2,))

        first.start()
        self.assertTrue(blocking_node.started.wait(timeout=1.0))
        second.start()
        blocking_node.allow_finish.set()

        first.join(timeout=1.0)
        second.join(timeout=1.0)

        self.assertFalse(errors)
        self.assertEqual(blocking_node.create_calls, 1)
        self.assertIn(1, ProxySubscriberCached._topics['/topic']['subscribers'])
        self.assertIn(2, ProxySubscriberCached._topics['/topic']['subscribers'])

    def test_concurrent_subscribe_propagates_setup_failure_to_waiters(self):
        """Concurrent waiters should fail if the in-flight subscription setup fails."""
        failing_node = _FailingSubscriptionNode()
        ProxySubscriberCached._node = failing_node
        proxy = ProxySubscriberCached()
        errors = []

        def _subscribe():
            try:
                with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localinfo'):
                    proxy.subscribe('/topic', _FakeMsg)
            except Exception as exc:  # pylint: disable=W0703
                errors.append(exc)

        first = threading.Thread(target=_subscribe)
        second = threading.Thread(target=_subscribe)

        first.start()
        self.assertTrue(failing_node.started.wait(timeout=1.0))
        second.start()
        failing_node.allow_finish.set()

        first.join(timeout=1.0)
        second.join(timeout=1.0)

        self.assertEqual(len(errors), 2)
        self.assertTrue(all(isinstance(exc, RuntimeError) for exc in errors))
        self.assertNotIn('/topic', ProxySubscriberCached._topics)

    def test_failed_subscribe_does_not_leave_topic_available(self):
        """A single failed subscribe should clean up its placeholder topic entry."""
        failing_node = _FailingSubscriptionNode()
        ProxySubscriberCached._node = failing_node
        proxy = ProxySubscriberCached()
        failing_node.allow_finish.set()

        with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localinfo'):
            with self.assertRaises(RuntimeError):
                proxy.subscribe('/topic', _FakeMsg)

        self.assertFalse(ProxySubscriberCached.is_available('/topic'))

    def test_callback_ignores_topic_removed_before_lock_is_acquired(self):
        """Late callbacks during unsubscribe should exit cleanly if the topic disappears."""
        topic = '/topic'
        ProxySubscriberCached._topics[topic] = {'subscription': _FakeSubscription(_FakeMsg),
                                                'ready_event': threading.Event(),
                                                'setup_error': None,
                                                'last_msg': None,
                                                'buffered': False,
                                                'msg_queue': [],
                                                'callbacks': {},
                                                'callback_items': (),
                                                'subscribers': [1]}
        original_lock = ProxySubscriberCached._subscription_lock
        ProxySubscriberCached._subscription_lock = _HookedLock(lambda: ProxySubscriberCached._topics.pop(topic, None))
        try:
            with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localinfo') as log_info:
                ProxySubscriberCached._callback(_FakeMsg('late'), topic)
            log_info.assert_called_once()
        finally:
            ProxySubscriberCached._subscription_lock = original_lock

    def test_buffer_accessors_ignore_topic_removed_before_lock_is_acquired(self):
        """Buffer/cache readers should exit cleanly if the topic disappears before lock acquisition."""
        topic = '/topic'
        ProxySubscriberCached._topics[topic] = {'subscription': _FakeSubscription(_FakeMsg),
                                                'ready_event': threading.Event(),
                                                'setup_error': None,
                                                'last_msg': _FakeMsg('cached'),
                                                'buffered': True,
                                                'msg_queue': [_FakeMsg('queued')],
                                                'callbacks': {},
                                                'callback_items': (),
                                                'subscribers': [1]}
        original_lock = ProxySubscriberCached._subscription_lock
        ProxySubscriberCached._subscription_lock = _HookedLock(lambda: ProxySubscriberCached._topics.pop(topic, None))
        try:
            self.assertFalse(ProxySubscriberCached.has_msg(topic))
            self.assertFalse(ProxySubscriberCached.has_buffered(topic))
            self.assertIsNone(ProxySubscriberCached.get_from_buffer(topic))
            self.assertIsNone(ProxySubscriberCached.peek_at_buffer(topic))
        finally:
            ProxySubscriberCached._subscription_lock = original_lock

    def test_set_callback_replaces_existing_callback(self):
        """set_callback should replace the existing callback for the same subscriber instance."""
        topic = '/topic'
        ProxySubscriberCached._node = type('Node', (), {'executor': _FakeExecutor()})()
        first = []
        second = []

        def _first(msg):
            first.append(msg.data)

        def _second(msg):
            second.append(msg.data)

        ProxySubscriberCached._topics[topic] = {'subscription': _FakeSubscription(_FakeMsg),
                                                'ready_event': threading.Event(),
                                                'setup_error': None,
                                                'last_msg': None,
                                                'buffered': False,
                                                'msg_queue': [],
                                                'callbacks': {1: _first},
                                                'callback_items': ((1, _first),),
                                                'subscribers': [1]}

        with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localinfo'):
            ProxySubscriberCached.set_callback(topic, _second, 1)

        ProxySubscriberCached._callback(_FakeMsg('payload'), topic)
        self.assertFalse(first)
        self.assertEqual(second, ['payload'])

    def test_unsubscribe_topic_keeps_shared_subscription_until_last_subscriber(self):
        """unsubscribe_topic should remove one subscriber without destroying the shared topic."""
        topic = '/topic'
        ProxySubscriberCached._node = type('Node', (), {'executor': _FakeExecutor()})()
        ProxySubscriberCached._topics[topic] = {'subscription': _FakeSubscription(_FakeMsg),
                                                'ready_event': threading.Event(),
                                                'setup_error': None,
                                                'last_msg': None,
                                                'buffered': False,
                                                'msg_queue': [],
                                                'callbacks': {1: lambda _msg: None, 2: lambda _msg: None},
                                                'callback_items': (),
                                                'subscribers': [1, 2]}
        ProxySubscriberCached._refresh_callback_items(ProxySubscriberCached._topics[topic])

        with patch('flexbe_core.proxy.proxy_subscriber_cached.Logger.localdebug'):
            ProxySubscriberCached.unsubscribe_topic(topic, inst_id=1)

        self.assertIn(topic, ProxySubscriberCached._topics)
        self.assertEqual(ProxySubscriberCached._topics[topic]['subscribers'], [2])
        self.assertNotIn(1, ProxySubscriberCached._topics[topic]['callbacks'])

    def test_make_persistant_prevents_remove_last_msg(self):
        """Persistent topics should ignore remove_last_msg requests."""
        topic = '/topic'
        ProxySubscriberCached._topics[topic] = {'subscription': _FakeSubscription(_FakeMsg),
                                                'ready_event': threading.Event(),
                                                'setup_error': None,
                                                'last_msg': _FakeMsg('cached'),
                                                'buffered': True,
                                                'msg_queue': [_FakeMsg('queued')],
                                                'callbacks': {},
                                                'callback_items': (),
                                                'subscribers': [1]}
        ProxySubscriberCached.make_persistant(topic)

        ProxySubscriberCached.remove_last_msg(topic, clear_buffer=True)

        self.assertEqual(ProxySubscriberCached.get_last_msg(topic).data, 'cached')
        self.assertTrue(ProxySubscriberCached.has_buffered(topic))


if __name__ == '__main__':
    unittest.main()
