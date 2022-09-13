#!/usr/bin/env python
import rclpy
from threading import Timer

from flexbe_core.logger import Logger
from flexbe_core.proxy.qos import QOS_DEFAULT


class ProxyServiceCaller(object):
    """
    A proxy for calling services.
    """
    _node = None
    _services = {}

    _result = {}

    @staticmethod
    def _initialize(node):
        ProxyServiceCaller._node = node
        Logger.initialize(node)

    def __init__(self, topics={}, wait_duration=10):
        """
        Initializes the proxy with optionally a given set of clients.

        @type topics: dictionary string - message class
        @param topics: A dictionary containing a collection of topic - message type pairs.

        @type wait_duration: float
        @param wait_duration: Defines how long to wait (seconds) for the given services if not available right now.
        """
        for topic, srv_type in topics.items():
            self.setupService(topic, srv_type, wait_duration)

    def setupService(self, topic, srv_type, wait_duration=10):
        """
        Tries to set up a service caller for calling it later.

        @type topic: string
        @param topic: The topic of the service to call.

        @type srv_type: service class
        @param srv_type: The type of messages of this service.

        @type wait_duration: float
        @param wait_duration: Defines how long to wait for the given service if it is not available right now.
        """
        if topic not in ProxyServiceCaller._services:
            Logger.localinfo(f'Set up ProxyServiceCaller for new topic {topic} ...')
            ProxyServiceCaller._services[topic] = ProxyServiceCaller._node.create_client(srv_type, topic)
            if isinstance(wait_duration, float):
                self._check_service_available(topic, wait_duration)

        else:
            if srv_type is not ProxyServiceCaller._services[topic].srv_type:
                if srv_type.__name__ == ProxyServiceCaller._services[topic].srv_type.__name__:
                    Logger.localinfo(f'Existing service for {topic} with same message type name, but different instance - re-create service!')
                    ProxyServiceCaller._node.destroy_client(ProxyServiceCaller._services[topic])
                    ProxyServiceCaller._services[topic] = ProxyServiceCaller._node.create_client(srv_type, topic)
                    if isinstance(wait_duration, float):
                        self._check_service_available(topic, wait_duration)
                else:
                    raise TypeError("Trying to replace existing service caller with different service msg type")

    def is_available(self, topic, wait_duration=1.0):
        """
        Checks if the service on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        @type wait_duration: float
        @param wait_duration: Seconds to wait for service to become available (default: 1.0)
        """
        return self._check_service_available(topic, wait_duration)

    def call(self, topic, request, wait_duration=1.0):
        """
        Performs a service call on the given topic.

        @type topic: string
        @param topic: The topic to call.

        @type request: service
        @param request: The request to send to this service.

        @type wait_duration: float
        @param wait_duration: Seconds to wait for service to become available (default: 1.0)
        """
        if not self._check_service_available(topic, wait_duration):
            raise ValueError('Cannot call service client %s: Topic not available.' % topic)
        # call service (forward any exceptions)

        if not isinstance(request, ProxyServiceCaller._services[topic].srv_type.Request):
            if request.__class__.__name__ == ProxyServiceCaller._services[topic].srv_type.Request.__name__:
                # This is the case if the same class is imported multiple times
                # To avoid rclpy TypeErrors, we will Automatically convert the to base type
                # used in the service clients

                new_request = ProxyServiceCaller._services[topic].srv_type.Request()
                for attr, val in vars(new_request):
                    assert hasattr(request, attr), "Types must share common attributes!"  # Validate that attributes in common

                for attr, val in vars(request):
                    setattr(new_request, attr, val)
            else:
                raise TypeError(f"Invalid request type {request.__class__.__name__} (vs. {ProxyServiceCaller._services[topic].srv_type.Request.__name__}) for topic {topic}")
        else:
            # Same class definition instance as stored
            new_request = request

        Logger.loginfo("Client about to call service")

        return ProxyServiceCaller._services[topic].call(new_request)

    def call_async(self, topic, request, wait_duration=1.0):
        """
        Performs an asynchronous service call on the given topic.
        Completion and result can be checked via the "done" and "result" methods.

        @type topic: string
        @param topic: The topic to call.

        @type request: service
        @param request: The request to send to this service.

        @type wait_duration: float
        @param wait_duration: Seconds to wait for service to become available (default: 1.0)
        """
        if not self._check_service_available(topic, wait_duration):
            raise ValueError('Cannot call service client %s: Topic not available.' % topic)

        # call service (forward any exceptions)
        if not isinstance(request, ProxyServiceCaller._services[topic].srv_type.Request):
            if request.__class__.__name__ == ProxyServiceCaller._services[topic].srv_type.Request.__name__:
                # This is the case if the same class is imported multiple times
                # To avoid rclpy TypeErrors, we will automatically convert to the base type
                # used in the original service clients

                new_request = ProxyServiceCaller._services[topic].srv_type.Request()
                assert new_request.__slots__ == request.__slots__, f"Message attributes for {topic} do not match!"
                for attr in request.__slots__:
                    setattr(new_request, attr, getattr(request, attr))
            else:
                raise TypeError(f"Invalid request type {request.__class__.__name__} (vs. {ProxyServiceCaller._services[topic].srv_type.Request.__name__}) for topic {topic}")
        else:
            # Same class definition instance as stored
            new_request = request

        ProxyServiceCaller._result[topic] = ProxyServiceCaller._services[topic].call_async(new_request)

    def done(self, topic):
        """
        Check whether a service call on the given topic has already completed.

        @type topic: string
        @param topic: The topic to call.
        """
        if topic not in ProxyServiceCaller._result:
            return False
        return ProxyServiceCaller._result[topic].done()

    def result(self, topic):
        """
        Obtain the result of a completed service call on the given topic.

        @type topic: string
        @param topic: The topic to call.
        """
        if not self.done(topic):
            return None
        return ProxyServiceCaller._result[topic].result()

    def _check_service_available(self, topic, wait_duration=1):
        """
        Checks whether a service is available.

        @type topic: string
        @param topic: The topic of the service.

        @type wait_duration: float
        @param wait_duration: Defines how long to wait for the given service if it is not available right now.
        """
        client = ProxyServiceCaller._services.get(topic)
        if client is None:
            Logger.error(f"Service client {topic} not yet registered, need to add it first!")
            return False

        if not isinstance(wait_duration, float):
            Logger.localinfo(f"Check for service {topic} requires floating point wait_duration in seconds (change to 0.001)!")
            wait_duration = 0.001

        warning_sent = False
        available = False
        wait_timer = None
        try:
            if wait_duration > 0.5:
                # Only start warning timer if significant wait duration
                wait_timer = Timer(0.5, self._print_wait_warning, [topic])
                wait_timer.start()

            available = client.wait_for_service(wait_duration)
        except rclpy.exceptions.ROSInterruptException:
            available = False

        if wait_timer:
            try:
                wait_timer.cancel()
            except Exception:
                # already printed the warning
                warning_sent = True

        if not available:
            Logger.error(f"Service client {topic} not available! (timed out with wait_duration={wait_duration:.3f} seconds)")
        elif warning_sent:
            Logger.info("Finally found service %s..." % (topic))

        return available

    def _print_wait_warning(self, topic):
        Logger.warning("Waiting for service %s..." % (topic))
