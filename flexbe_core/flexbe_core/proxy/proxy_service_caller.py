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

    def __init__(self, topics={}, qos=None, wait_duration=10):
        """
        Initializes the proxy with optionally a given set of clients.

        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.

        @type persistent: bool
        @param persistent: Defines if the service callers are persistent.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given services if not available right now.
        """
        for topic, msg_type in topics.items():
            self.setupService(topic, msg_type, qos, wait_duration)

    def setupService(self, topic, msg_type, qos=QOS_DEFAULT, wait_duration=10):
        """
        Tries to set up a service caller for calling it later.

        @type topic: string
        @param topic: The topic of the service to call.

        @type msg_type: service class
        @param msg_type: The type of messages of this service.

        @type persistent: bool
        @param persistent: Defines if this service caller is persistent.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given service if it is not available right now.
        """
        if topic not in ProxyServiceCaller._services:
            services = self._node.get_service_names_and_types()
            found_service = False
            for i in range(len(services)):
                if services[i][0] == topic:
                    found_service = True
                    break

            if found_service:
                ProxyServiceCaller._services[topic] = ProxyServiceCaller._node.create_client(msg_type, topic)
                self._check_service_available(topic, wait_duration)
        else:
            if not isinstance(msg_type, ProxyServiceCaller._services[topic].srv_type):
                if msg_type.__name__ == ProxyServiceCaller._services[topic].srv_type.__name__:
                    ProxyServiceCaller._node.destroy_client(ProxyServiceCaller._services[topic])
                    ProxyServiceCaller._services[topic] = ProxyServiceCaller._node.create_client(msg_type, topic)

    def is_available(self, topic):
        """
        Checks if the service on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return self._check_service_available(topic)

    def call(self, topic, request):
        """
        Performs a service call on the given topic.

        @type topic: string
        @param topic: The topic to call.

        @type request: service
        @param request: The request to send to this service.
        """
        if not self._check_service_available(topic):
            raise ValueError('Cannot call service client %s: Topic not available.' % topic)
        # call service (forward any exceptions)
        Logger.loginfo("Client about to call service")
        return ProxyServiceCaller._services[topic].call(request)

    def call_async(self, topic, request):
        """
        Performs an asynchronous service call on the given topic.
        Completion and result can be checked via the "done" and "result" methods.

        @type topic: string
        @param topic: The topic to call.

        @type request: service
        @param request: The request to send to this service.
        """
        if not self._check_service_available(topic):
            raise ValueError('Cannot call service client %s: Topic not available.' % topic)
        # call service (forward any exceptions)
        ProxyServiceCaller._result[topic] = ProxyServiceCaller._services[topic].call_async(request)

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

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given service if it is not available right now.
        """
        client = ProxyServiceCaller._services.get(topic)
        if client is None:
            Logger.error("Service client %s not yet registered, need to add it first!" % topic)
            return False
        warning_sent = False
        available = False
        try:
            t = Timer(0.5, self._print_wait_warning, [topic])
            t.start()
            client.wait_for_service(wait_duration)
            # rospy.wait_for_service(topic, wait_duration)
            available = True
        except rclpy.exceptions.ROSInterruptException:
            available = False

        try:
            t.cancel()
        except Exception:
            # already printed the warning
            warning_sent = True

        if not available:
            Logger.error("Service client %s timed out!" % topic)
            return False
        else:
            if warning_sent:
                Logger.info("Finally found service %s..." % (topic))

        return True

    def _print_wait_warning(self, topic):
        Logger.warning("Waiting for service %s..." % (topic))
