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

"""Unit tests for ProxyPublisher teardown races."""

import unittest
from unittest.mock import patch

from flexbe_core.core.exceptions import ProxyTypeError
from flexbe_core.core.topics import Topics
from flexbe_core.proxy.proxy_publisher import ProxyPublisher


class _FakeMsg:
    """Minimal message object for publisher tests."""

    __slots__ = ('data',)

    def __init__(self, data):
        self.data = data


class _FakePublisher:
    """Publisher handle exposing msg_type and publish()."""

    def __init__(self, msg_type=_FakeMsg, subscribers=0):
        self.msg_type = msg_type
        self.messages = []
        self.subscribers = subscribers

    def publish(self, msg):
        """Record published messages."""
        self.messages.append(msg)

    def get_subscription_count(self):
        """Return the configured subscriber count."""
        return self.subscribers


class _HookedLock:
    """Context manager that runs a hook before entering the protected section."""

    def __init__(self, hook):
        self._hook = hook

    def __enter__(self):
        self._hook()
        return self

    def __exit__(self, exc_type, exc, tb):
        return False


class _FakeExecutor:
    """Record scheduled destruction tasks."""

    def __init__(self):
        self.tasks = []

    def create_task(self, callback, *args):
        self.tasks.append((callback, args))


class _FakeNode:
    """Create fake publishers and expose an executor."""

    def __init__(self):
        self.executor = _FakeExecutor()
        self.created_publishers = []
        self.destroyed_publishers = []

    def create_publisher(self, msg_type, _topic, _qos):
        publisher = _FakePublisher(msg_type=msg_type)
        self.created_publishers.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        """Record destroyed publishers and report success."""
        self.destroyed_publishers.append(publisher)
        return True


class TestProxyPublisher(unittest.TestCase):
    """Validate publisher behavior during concurrent teardown."""

    def setUp(self):
        """Reset proxy singleton state before each test."""
        ProxyPublisher._topics.clear()
        ProxyPublisher._node = None

    def test_publish_ignores_topic_removed_before_lock_is_acquired(self):
        """A late publish should exit cleanly if the topic disappears before the lock is taken."""
        topic = '/topic'
        ProxyPublisher._topics[topic] = {'publisher': _FakePublisher(), 'count': 1}
        original_lock = ProxyPublisher._publisher_sync_lock
        ProxyPublisher._publisher_sync_lock = _HookedLock(lambda: ProxyPublisher._topics.pop(topic, None))
        try:
            with patch('flexbe_core.proxy.proxy_publisher.Logger.warning') as log_warning:
                ProxyPublisher.publish(topic, _FakeMsg('late'))
            log_warning.assert_called_once()
        finally:
            ProxyPublisher._publisher_sync_lock = original_lock

    def test_create_publisher_recreates_same_name_type(self):
        """Same-name message reloads should recreate the publisher and reset the refcount."""
        topic = '/topic'
        fake_node = _FakeNode()
        MsgTypeA = type('ReloadedMsg', (), {'__slots__': ('data',)})
        MsgTypeB = type('ReloadedMsg', (), {'__slots__': ('data',)})
        ProxyPublisher._node = fake_node
        ProxyPublisher._topics[topic] = {'publisher': _FakePublisher(msg_type=MsgTypeA), 'count': 2}

        with patch('flexbe_core.proxy.proxy_publisher.Logger.localwarn'):
            ProxyPublisher.create_publisher(topic, MsgTypeB)

        self.assertIs(ProxyPublisher._topics[topic]['publisher'].msg_type, MsgTypeB)
        self.assertEqual(ProxyPublisher._topics[topic]['count'], 1)
        self.assertEqual(len(fake_node.executor.tasks), 1)

    def test_create_publisher_rejects_different_type_name(self):
        """Different message type names should be rejected instead of replacing the publisher."""
        topic = '/topic'
        ProxyPublisher._topics[topic] = {'publisher': _FakePublisher(msg_type=_FakeMsg), 'count': 1}

        with patch('flexbe_core.proxy.proxy_publisher.Logger.info'):
            with self.assertRaises(ProxyTypeError):
                ProxyPublisher.create_publisher(topic, type('OtherMsg', (), {'__slots__': ('data',)}))

    def test_create_and_remove_publisher_updates_reference_count(self):
        """Repeated create/remove calls should honor publisher reference counting."""
        topic = '/topic'
        fake_node = _FakeNode()
        ProxyPublisher._node = fake_node

        ProxyPublisher.create_publisher(topic, _FakeMsg)
        ProxyPublisher.create_publisher(topic, _FakeMsg)
        self.assertEqual(ProxyPublisher._topics[topic]['count'], 2)

        with patch('flexbe_core.proxy.proxy_publisher.Logger.localdebug'):
            ProxyPublisher.remove_publisher(topic)
        self.assertIn(topic, ProxyPublisher._topics)
        self.assertEqual(ProxyPublisher._topics[topic]['count'], 1)

        with patch('flexbe_core.proxy.proxy_publisher.Logger.localdebug'):
            ProxyPublisher.remove_publisher(topic)
        self.assertNotIn(topic, ProxyPublisher._topics)
        self.assertEqual(len(fake_node.executor.tasks), 1)

    def test_initialize_and_shutdown_reset_topics_and_outcome_counters(self):
        """Initialization and shutdown should wire the node, clear topics, and reset counters."""
        topic = '/topic'
        fake_node = _FakeNode()
        ProxyPublisher._topics[topic] = {'publisher': _FakePublisher(), 'count': 1}
        ProxyPublisher._outcome_publish_total = 4
        ProxyPublisher._outcome_publish_nonzero = 3
        with patch('flexbe_core.proxy.proxy_publisher.Logger.initialize') as logger_initialize:
            ProxyPublisher.initialize(fake_node)

        logger_initialize.assert_called_once_with(fake_node)
        self.assertIs(ProxyPublisher._node, fake_node)
        self.assertEqual(ProxyPublisher.get_outcome_publish_counters(), {
            'outcome_pub_total': 0,
            'outcome_pub_nonzero': 0,
        })

        ProxyPublisher._topics[topic] = {'publisher': _FakePublisher(), 'count': 1}
        with patch('flexbe_core.proxy.proxy_publisher.Logger.error') as log_error:
            ProxyPublisher.shutdown()

        self.assertEqual(ProxyPublisher._topics, {})
        self.assertEqual(len(fake_node.destroyed_publishers), 1)
        log_error.assert_not_called()

    def test_publish_converts_reloaded_message_instances_and_tracks_outcomes(self):
        """Publishing a reloaded message class should convert it and update outcome counters."""
        topic = Topics._OUTCOME_TOPIC
        MsgTypeA = type('ReloadedMsg', (), {'__slots__': ('data',)})
        MsgTypeB = type('ReloadedMsg', (), {'__slots__': ('data',)})
        publisher = _FakePublisher(msg_type=MsgTypeA)
        ProxyPublisher._topics[topic] = {'publisher': publisher, 'count': 1}
        ProxyPublisher.reset_outcome_publish_counters()

        message = MsgTypeB()
        message.data = 0

        with patch('flexbe_core.proxy.proxy_publisher.Logger.localinfo'):
            ProxyPublisher.publish(topic, message)

        self.assertEqual(len(publisher.messages), 1)
        self.assertIsInstance(publisher.messages[0], MsgTypeA)
        self.assertEqual(publisher.messages[0].data, 0)
        self.assertEqual(ProxyPublisher.get_outcome_publish_counters(), {
            'outcome_pub_total': 1,
            'outcome_pub_nonzero': 0,
        })

    def test_number_of_subscribers_and_wait_for_any_cover_missing_success_and_timeout_paths(self):
        """Subscriber helpers should report missing topics, success after warning, and timeout failures."""
        topic = '/topic'
        publisher = _FakePublisher(subscribers=1)
        ProxyPublisher._topics[topic] = {'publisher': publisher, 'count': 1}

        with patch('flexbe_core.proxy.proxy_publisher.Logger.error') as log_error:
            self.assertEqual(ProxyPublisher.number_of_subscribers('/missing'), -1)
        log_error.assert_called_once()
        self.assertEqual(ProxyPublisher.number_of_subscribers(topic), 1)

        class _FakeTimer:

            def __init__(self, _delay, callback, args):
                self._callback = callback
                self._args = args

            def start(self):
                self._callback(*self._args)

            def cancel(self):
                return None

        with patch('flexbe_core.proxy.proxy_publisher.Timer', _FakeTimer), \
                patch.object(ProxyPublisher, '_wait_for_subscribers', return_value=True), \
                patch('flexbe_core.proxy.proxy_publisher.Logger.info') as info_log, \
                patch('flexbe_core.proxy.proxy_publisher.Logger.warning'):
            self.assertTrue(ProxyPublisher.wait_for_any(topic, timeout=0.1))
        info_log.assert_called_once()

        with patch.object(ProxyPublisher, '_wait_for_subscribers', return_value=False), \
                patch('flexbe_core.proxy.proxy_publisher.Logger.error') as error_log:
            self.assertFalse(ProxyPublisher.wait_for_any(topic, timeout=0.1))
        error_log.assert_called_once()

    def test_destroy_publisher_reports_false_and_exception_results(self):
        """Destroying publishers should log both unsuccessful returns and thrown exceptions."""
        fake_node = _FakeNode()
        ProxyPublisher._node = fake_node
        publisher = _FakePublisher()

        fake_node.destroy_publisher = lambda pub: False
        with patch('flexbe_core.proxy.proxy_publisher.Logger.localwarn') as localwarn:
            ProxyPublisher.destroy_publisher(publisher, '/topic')
        localwarn.assert_called_once()

        fake_node.destroy_publisher = lambda pub: (_ for _ in ()).throw(RuntimeError('boom'))
        with patch('flexbe_core.proxy.proxy_publisher.Logger.error') as log_error:
            ProxyPublisher.destroy_publisher(publisher, '/topic')
        log_error.assert_called_once()


if __name__ == '__main__':
    unittest.main()
