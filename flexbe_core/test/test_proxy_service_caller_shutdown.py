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

"""Unit tests for ProxyServiceCaller shutdown behavior."""

import unittest
from unittest.mock import patch

from flexbe_core.core.exceptions import ProxyTypeError
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

import rclpy


class _FakeNode:
    """Track service clients passed to destroy_client."""

    def __init__(self):
        self.destroyed = []
        self.created_clients = []
        self.executor = _FakeExecutor()

    def destroy_client(self, client):
        """Record the destroyed client."""
        self.destroyed.append(client)
        return True

    def create_client(self, srv_type, _topic):
        """Create a fake service client for the requested service type."""
        client = _FakeServiceClient(srv_type)
        self.created_clients.append(client)
        return client


class _FakeFuture:
    """Minimal async future used by service caller tests."""

    def __init__(self, result_obj=None, done=True):
        self._result_obj = result_obj
        self._done = done

    def done(self):
        """Return completion state."""
        return self._done

    def result(self):
        """Return stored future result."""
        return self._result_obj


class _FakeExecutor:
    """Run scheduled tasks immediately for deterministic tests."""

    def __init__(self):
        self.tasks = []

    def create_task(self, callback, *args):
        self.tasks.append((callback, args))


class _FakeServiceClient:
    """Minimal service client with conversion and async-call observability."""

    def __init__(self, srv_type):
        self.srv_type = srv_type
        self.sync_requests = []
        self.async_requests = []
        self.available = True
        self.wait_requests = []
        self.wait_side_effect = None

    def wait_for_service(self, _timeout):
        self.wait_requests.append(_timeout)
        if self.wait_side_effect is not None:
            raise self.wait_side_effect
        return self.available

    def call(self, request):
        self.sync_requests.append(request)
        return request

    def call_async(self, request):
        self.async_requests.append(request)
        return _FakeFuture(result_obj=request)


def _make_service_type(name):
    """Create a minimal service type with Request class."""
    Request = type(f'{name}Request', (), {'__slots__': ('value',)})
    return type(name, (), {'Request': Request})


class TestProxyServiceCallerShutdown(unittest.TestCase):
    """Validate shutdown cleanup and client destruction semantics."""

    def setUp(self):
        """Reset static proxy state before each test."""
        ProxyServiceCaller._services.clear()
        ProxyServiceCaller._results.clear()
        ProxyServiceCaller._node = None
        ProxyServiceCaller._service_generation_counter = 0

    def test_shutdown_destroys_service_client_and_clears_state(self):
        """Shutdown should destroy stored client handles and clear registries."""
        fake_node = _FakeNode()
        fake_client = object()
        ProxyServiceCaller._node = fake_node
        ProxyServiceCaller._services['/service'] = {'service': fake_client, 'count': 1}
        ProxyServiceCaller._results['/service'] = object()

        with patch('builtins.print'):
            ProxyServiceCaller.shutdown()

        self.assertEqual(fake_node.destroyed, [fake_client])
        self.assertFalse(ProxyServiceCaller._services)
        self.assertFalse(ProxyServiceCaller._results)

    def test_done_and_result_ignore_stale_future_after_service_recreate(self):
        """A recreated service client should not expose the prior async future."""
        topic = '/service'
        stale_future = _FakeFuture(result_obj='stale-result', done=True)
        ProxyServiceCaller._services[topic] = {'service': object(), 'generation': 2, 'count': 1}
        ProxyServiceCaller._results[topic] = {'future': stale_future, 'generation': 1}

        self.assertFalse(ProxyServiceCaller.done(topic))
        self.assertIsNone(ProxyServiceCaller.result(topic))

    def test_call_converts_same_name_request_type(self):
        """Sync service calls should convert reloaded request classes with the same name."""
        topic = '/service'
        fake_node = _FakeNode()
        fake_node.executor = _FakeExecutor()
        ServiceType = _make_service_type('SharedService')
        ProxyServiceCaller._node = fake_node
        client = _FakeServiceClient(ServiceType)
        ProxyServiceCaller._services[topic] = {'service': client, 'generation': 1, 'count': 1}

        ReloadedRequest = type(ServiceType.Request.__name__, (), {'__slots__': ('value',)})
        request = ReloadedRequest()
        request.value = 'payload'

        with patch('flexbe_core.proxy.proxy_service_caller.Logger.loginfo'):
            result = ProxyServiceCaller.call(topic, request, wait_duration=0.01)

        self.assertIsInstance(client.sync_requests[0], ServiceType.Request)
        self.assertEqual(client.sync_requests[0].value, 'payload')
        self.assertIs(result, client.sync_requests[0])

    def test_call_async_recreates_same_name_service_type(self):
        """Same-name service reloads should recreate the client and reset refcount."""
        topic = '/service'
        fake_node = _FakeNode()
        fake_node.executor = _FakeExecutor()
        fake_node.created_clients = []
        ServiceTypeA = _make_service_type('SharedService')
        ServiceTypeB = _make_service_type('SharedService')
        old_client = _FakeServiceClient(ServiceTypeA)
        ProxyServiceCaller._node = fake_node
        ProxyServiceCaller._services[topic] = {'service': old_client, 'generation': 1, 'count': 2}

        with patch('flexbe_core.proxy.proxy_service_caller.Logger.localwarn'):
            ProxyServiceCaller.setup_service(topic, ServiceTypeB, wait_duration=0.01)

        self.assertIs(ProxyServiceCaller._services[topic]['service'].srv_type, ServiceTypeB)
        self.assertEqual(ProxyServiceCaller._services[topic]['count'], 1)
        self.assertEqual(len(fake_node.executor.tasks), 1)

    def test_setup_service_rejects_different_type_name(self):
        """Different service type names should raise instead of replacing the client."""
        topic = '/service'
        ServiceType = _make_service_type('SharedService')
        ProxyServiceCaller._services[topic] = {'service': _FakeServiceClient(ServiceType), 'generation': 1, 'count': 1}

        with self.assertRaises(ProxyTypeError):
            ProxyServiceCaller.setup_service(topic, _make_service_type('OtherService'), wait_duration=0.01)

    def test_check_service_available_handles_missing_non_numeric_and_interrupt_cases(self):
        """Availability checks should cover missing clients, bad wait values, and ROS interrupts."""
        with patch('flexbe_core.proxy.proxy_service_caller.Logger.error') as error_log:
            self.assertFalse(ProxyServiceCaller._check_service_available('/missing', wait_duration=0.1))
        error_log.assert_called_once()

        topic = '/service'
        client = _FakeServiceClient(_make_service_type('SharedService'))
        ProxyServiceCaller._services[topic] = {'service': client, 'generation': 1, 'count': 1}

        with patch('flexbe_core.proxy.proxy_service_caller.Logger.localwarn') as localwarn:
            self.assertTrue(ProxyServiceCaller._check_service_available(topic, wait_duration='bad'))
        self.assertEqual([0.001], client.wait_requests)
        localwarn.assert_called_once()

        client.wait_requests.clear()
        client.wait_side_effect = rclpy.exceptions.ROSInterruptException()
        with patch('flexbe_core.proxy.proxy_service_caller.Logger.error') as error_log:
            self.assertFalse(ProxyServiceCaller._check_service_available(topic, wait_duration=0.1))
        self.assertEqual([0.1], client.wait_requests)
        error_log.assert_called_once()

    def test_check_service_available_reports_delayed_success_after_wait_warning(self):
        """A delayed available service should emit the final success log after the wait warning fires."""
        topic = '/service'
        client = _FakeServiceClient(_make_service_type('SharedService'))
        ProxyServiceCaller._services[topic] = {'service': client, 'generation': 1, 'count': 1}

        class _ImmediateTimer:

            def __init__(self, _delay, callback, args):
                self._callback = callback
                self._args = args

            def start(self):
                self._callback(*self._args)

            def cancel(self):
                return None

        with patch('flexbe_core.proxy.proxy_service_caller.Timer', _ImmediateTimer), \
                patch('flexbe_core.proxy.proxy_service_caller.Logger.info') as info_log, \
                patch('flexbe_core.proxy.proxy_service_caller.Logger.warning'):
            self.assertTrue(ProxyServiceCaller._check_service_available(topic, wait_duration=1.0))

        self.assertEqual([1.0], client.wait_requests)
        info_log.assert_called_once()

    def test_remove_client_and_destroy_service_cover_remaining_cleanup_branches(self):
        """Removing clients and destroying services should cover retain, warn, and exception paths."""
        topic = '/service'
        fake_node = _FakeNode()
        ProxyServiceCaller._node = fake_node
        client = _FakeServiceClient(_make_service_type('SharedService'))
        ProxyServiceCaller._services[topic] = {'service': client, 'generation': 1, 'count': 2}

        with patch('flexbe_core.proxy.proxy_service_caller.Logger.localdebug') as localdebug:
            ProxyServiceCaller.remove_client(topic)

        self.assertIn(topic, ProxyServiceCaller._services)
        self.assertEqual(1, ProxyServiceCaller._services[topic]['count'])
        localdebug.assert_called_once()

        ProxyServiceCaller._results[topic] = {'future': object(), 'generation': 1}
        with patch('flexbe_core.proxy.proxy_service_caller.Logger.localdebug'):
            ProxyServiceCaller.remove_client(topic)

        self.assertNotIn(topic, ProxyServiceCaller._services)
        self.assertNotIn(topic, ProxyServiceCaller._results)
        self.assertEqual(1, len(fake_node.executor.tasks))

        fake_node.destroy_client = lambda _srv: False
        with patch('flexbe_core.proxy.proxy_service_caller.Logger.localwarn') as localwarn:
            ProxyServiceCaller.destroy_service(object(), topic)
        localwarn.assert_called_once()

        def _raise_destroy(_srv):
            raise RuntimeError('destroy boom')

        fake_node.destroy_client = _raise_destroy
        with patch('flexbe_core.proxy.proxy_service_caller.Logger.error') as error_log:
            ProxyServiceCaller.destroy_service(object(), topic)
        error_log.assert_called_once()


if __name__ == '__main__':
    unittest.main()
