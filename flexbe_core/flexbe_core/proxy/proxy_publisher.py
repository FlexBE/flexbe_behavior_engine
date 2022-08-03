import rclpy
import time
from threading import Timer

from flexbe_core.logger import Logger
from flexbe_core.proxy.qos import QOS_DEFAULT


class ProxyPublisher(object):
    """
    A proxy for publishing topics.
    """
    _node = None
    _topics = {}

    def _initialize(node):
        ProxyPublisher._node = node
        Logger.initialize(node)

    def __init__(self, topics={}, qos=None, **kwargs):
        """
        Initializes the proxy with optionally a given set of topics.
        Automatically creates a publisher for sending status messages.

        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.

        @type _latch: bool
        @param: _latch: Defines if messages on the given topics should be latched.

        @type _queue_size: int
        @param: _queue_size: Defines the queue size of the new publishers.
        """
        for topic, msg_type in topics.items():
            self.createPublisher(topic, msg_type, qos, **kwargs)

    def createPublisher(self, topic, msg_type, qos=None, **kwargs):
        """
        Adds a new publisher to the proxy.

        @type topic: string
        @param topic: The topic to publish on.

        @type msg_type: a message class
        @param msg_type: The type of messages of this topic.
        """
        if '_latch' in kwargs or '_queue_size' in kwargs:
            Logger.warning('DEPRECATED use of arguments in publisher')

        if topic not in ProxyPublisher._topics:
            qos = qos or QOS_DEFAULT
            ProxyPublisher._topics[topic] = ProxyPublisher._node.create_publisher(msg_type, topic, qos)
        else:
            if msg_type is not ProxyPublisher._topics[topic].msg_type:
                # Change in required msg_type for topic name  - update publisher with new type
                if msg_type.__name__ == ProxyPublisher._topics[topic].msg_type.__name__:
                    # Same message type name, so likely due to reloading Python module on behavior change
                    Logger.localinfo(f'Existing publisher for {topic} with same message type name, but different instance - re-create publisher!')
                    # crashes Humble - ProxyPublisher._node.destroy_publisher(ProxyPublisher._topics[topic])
                    qos = qos or QOS_DEFAULT
                    ProxyPublisher._topics[topic] = ProxyPublisher._node.create_publisher(msg_type, topic, qos)
                else:
                    Logger.info(f'Mis-matched msg_types ({msg_type.__name__} vs. {ProxyPublisher._topics[topic].msg_type.__name__}) for {topic}  (possibly due to reload of behavior)!')
                    raise TypeError("Trying to replace existing publisher with different msg type")

    def is_available(self, topic):
        """
        Checks if the publisher on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxyPublisher._topics

    def publish(self, topic, msg):
        """
        Publishes a message on the specified topic.

        @type topic: string
        @param topic: The topic to publish on.

        @type msg: message class (defined when created publisher)
        @param msg: The message to publish.
        """
        if topic not in ProxyPublisher._topics:
            Logger.warning('ProxyPublisher: topic %s not yet registered!' % topic)
            return

        if not isinstance(msg, ProxyPublisher._topics[topic].msg_type):
            # Change in required msg_type for topic name  - update publisher with new type
            if msg.__class__.__name__ == ProxyPublisher._topics[topic].msg_type.__name__:
            # This is the case if the same class is imported multiple times
            # To avoid rclpy TypeErrors, we will automatically convert to the base type
            # used in the original publisher
                Logger.localinfo('Publish - converting datatype for %s!\n%s: %s/%s' % (topic, str(msg.__class__.__name__),str(id(msg.__class__)),str(id(ProxyPublisher._topics[topic].msg_type))))

                new_msg = ProxyPublisher._topics[topic].msg_type()
                assert new_msg.__slots__ == msg.__slots__, f"Message attributes for {topic} do not match!"
                for attr in msg.__slots__:
                    setattr(new_msg, attr, getattr(msg, attr))

            else:
                raise TypeError(f"Invalid request type {msg.__class__.__name__} (vs. {ProxyPublisher._topics[topic].msg_type.__name__}) for topic {topic}")
        else:
            # Same class definition instance as stored
            new_msg = msg

        try:
            ProxyPublisher._topics[topic].publish(new_msg)
        except Exception as e:
            Logger.warning('Something went wrong when publishing to %s!\n%s: %s' % (topic, str(type(e)), str(e)))


    def wait_for_any(self, topic, timeout=5.0):
        """
        Blocks until there are any subscribers to the given topic.

        @type topic: string
        @param topic: The topic to publish on.

        @type timeout: float
        @param timeout: How many seconds should be the maximum blocked time.
        """
        pub = ProxyPublisher._topics.get(topic)
        if pub is None:
            Logger.error("Publisher %s not yet registered, need to add it first!" % topic)
            return False
        t = Timer(.5, self._print_wait_warning, [topic])
        t.start()
        available = self._wait_for_subscribers(pub, timeout)
        warning_sent = False
        try:
            t.cancel()
        except Exception:
            # already printed the warning
            warning_sent = True

        # Problem here
        if not available:
            Logger.error("Waiting for subscribers on %s timed out!" % topic)
            return False
        else:
            if warning_sent:
                Logger.info("Finally found subscriber on %s..." % (topic))
        return True

    def _print_wait_warning(self, topic):
        Logger.warning("Waiting for subscribers on %s..." % (topic))

    def _wait_for_subscribers(self, pub, timeout=5.0):
        starting_time = ProxyPublisher._node.get_clock().now()
        rate = ProxyPublisher._node.create_rate(100, ProxyPublisher._node.get_clock())

        while (ProxyPublisher._node.get_clock().now() - starting_time).nanoseconds * 10 ** -9 < timeout:
            if pub.get_subscription_count() > 0:
                return True

            rate.sleep()

        return False
