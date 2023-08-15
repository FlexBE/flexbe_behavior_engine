# Copyright 2023 Philipp Schillinger, Team ViGIR, Christopher Newport University
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


"""
A proxy for publishing topics.

Provides a single point for comminications for all states in behavior
"""

from threading import Timer, Event, Lock

from flexbe_core.logger import Logger
from flexbe_core.proxy.qos import QOS_DEFAULT


class ProxyPublisher:
    """A proxy for publishing topics."""

    _node = None
    _topics = {}
    _publisher_sync_lock = Lock()

    @staticmethod
    def initialize(node):
        """Initialize ROS setup for proxy publisher."""
        ProxyPublisher._node = node
        Logger.initialize(node)

    @staticmethod
    def shutdown():
        """Shuts this proxy down by resetting all publishers."""
        try:
            print(f"Shutdown proxy publisher with {len(ProxyPublisher._topics)} topics ...")
            for topic, pub in ProxyPublisher._topics.items():
                try:
                    ProxyPublisher._topics[topic] = None
                    ProxyPublisher._node.destroy_publisher(pub['publisher'])
                except Exception as exc:  # pylint: disable=W0703
                    Logger.error(f"Something went wrong during shutdown of proxy publisher for {topic}!\n%s %s",
                                 type(exc), str(exc))

            ProxyPublisher._topics.clear()

        except Exception as exc:  # pylint: disable=W0703
            Logger.error(f'Something went wrong during shutdown of proxy publisher !\n{str(exc)}')

    def __init__(self, topics=None, qos=None, **kwargs):
        """
        Initialize the proxy with optionally a given set of topics.

        Automatically creates a publisher for sending status messages.

        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.

        @type _latch: bool
        @param: _latch: Defines if messages on the given topics should be latched.

        @type _queue_size: int
        @param: _queue_size: Defines the queue size of the new publishers.
        """
        if topics is not None:
            for topic, msg_type in topics.items():
                ProxyPublisher.create_publisher(topic, msg_type, qos, **kwargs)

    @classmethod
    def createPublisher(cls, topic, msg_type, qos=None, **kwargs):
        Logger.localerr("Deprecated: Use ProxyPublisher.create_publisher instead!")
        cls.create_publisher(topic, msg_type, qos, **kwargs)

    @classmethod
    def create_publisher(cls, topic, msg_type, qos=None, **kwargs):
        """
        Add a new publisher to the proxy.

        @type topic: string
        @param topic: The topic to publish on.

        @type msg_type: a message class
        @param msg_type: The type of messages of this topic.
        """
        if '_latch' in kwargs or '_queue_size' in kwargs:
            Logger.warning('DEPRECATED use of arguments in publisher')

        with cls._publisher_sync_lock:
            if topic not in ProxyPublisher._topics:
                qos = qos or QOS_DEFAULT
                ProxyPublisher._topics[topic] = {'publisher': ProxyPublisher._node.create_publisher(msg_type, topic, qos),
                                                 'count': 1}
            else:
                if msg_type is not ProxyPublisher._topics[topic]['publisher'].msg_type:
                    # Change in required msg_type for topic name  - update publisher with new type
                    if msg_type.__name__ == ProxyPublisher._topics[topic]['publisher'].msg_type.__name__:
                        # Same message type name, so likely due to reloading Python module on behavior change
                        Logger.localwarn(f'Existing publisher for {topic} with same message type name,'
                                         ' but different instance - re-create publisher!')

                        pub = ProxyPublisher._topics[topic]['publisher']
                        ProxyPublisher._node.executor.create_task(ProxyPublisher.destroy_publisher, pub, topic)

                        qos = qos or QOS_DEFAULT
                        ProxyPublisher._topics[topic]['publisher'] = ProxyPublisher._node.create_publisher(msg_type, topic, qos)
                        ProxyPublisher._topics[topic]['count'] = 1
                    else:
                        Logger.info(f"Mis-matched msg_types ({msg_type.__name__} vs."
                                    f" {ProxyPublisher._topics[topic]['publisher'].msg_type.__name__}) for {topic}"
                                    f" (possibly due to reload of behavior)!")
                        raise TypeError(f"Trying to replace existing publisher with different msg type for {topic}")
                else:
                    ProxyPublisher._topics[topic]['count'] = ProxyPublisher._topics[topic]['count'] + 1

    @classmethod
    def remove_publisher(cls, topic):
        """
        Remove publisher from proxy.

        @type topic: string
        @param topic: The topic to publish on.
        """
        pub = None
        count = -1
        with cls._publisher_sync_lock:
            if topic in ProxyPublisher._topics:
                ProxyPublisher._topics[topic]['count'] = ProxyPublisher._topics[topic]['count'] - 1
                count = ProxyPublisher._topics[topic]['count']
                if count < 1:
                    pub = ProxyPublisher._topics[topic]['publisher']
                    ProxyPublisher._topics.pop(topic)

        if pub is not None:
            Logger.localdebug(f"Publisher for '{topic}' has {count} references remaining ({id(pub)}).")
            ProxyPublisher._node.executor.create_task(ProxyPublisher.destroy_publisher, pub, topic)
        else:
            Logger.localdebug(f"Publisher for '{topic}' remains with {count} references ({id(pub)})!")

    @classmethod
    def is_available(cls, topic):
        """
        Check if the publisher on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxyPublisher._topics

    @classmethod
    def publish(cls, topic, msg):
        """
        Publish a message on the specified topic.

        @type topic: string
        @param topic: The topic to publish on.

        @type msg: message class (defined when created publisher)
        @param msg: The message to publish.
        """
        if topic not in ProxyPublisher._topics:
            Logger.warning('ProxyPublisher: topic %s not yet registered!' % topic)
            return

        with cls._publisher_sync_lock:
            msg_type = ProxyPublisher._topics[topic]['publisher'].msg_type

        if not isinstance(msg, msg_type):
            # Change in required msg_type for topic name  - update publisher with new type
            if msg.__class__.__name__ == msg_type.__name__:
                # This is the case if the same class is imported multiple times
                # To avoid rclpy TypeErrors, we will automatically convert to the base type
                # used in the original publisher
                Logger.localinfo('Publish - converting datatype for '
                                 '%s!\n%s: %s/%s' % (topic, str(msg.__class__.__name__), str(id(msg.__class__)),
                                                     str(id(msg_type))))

                new_msg = msg_type()
                assert new_msg.__slots__ == msg.__slots__, f"Message attributes for {topic} do not match!"
                for attr in msg.__slots__:
                    setattr(new_msg, attr, getattr(msg, attr))

            else:
                raise TypeError(f"Invalid request type {msg.__class__.__name__}"
                                f" (vs. {msg_type.__name__}) for topic {topic}")
        else:
            # Same class definition instance as stored
            new_msg = msg

        try:
            with cls._publisher_sync_lock:
                ProxyPublisher._topics[topic]['publisher'].publish(new_msg)
        except Exception as exc:  # pylint: disable=W0703
            Logger.warning('Something went wrong when publishing to %s!\n%s: %s' % (topic, str(type(exc)), str(exc)))
            import traceback  # pylint: disable=C0415
            Logger.localinfo(traceback.format_exc().replace("%", "%%"))

    @classmethod
    def number_of_subscribers(cls, topic):
        """
        Return the current number of active subscribers to a given topic.

        @param topic  name of topic
        @return number of subscribers, or -1 if topic is not available
        """
        pub = ProxyPublisher._topics.get(topic)
        if pub is None:
            Logger.error("Publisher %s not yet registered, need to add it first!" % topic)
            return -1
        return pub['publisher'].get_subscription_count()

    @classmethod
    def wait_for_any(cls, topic, timeout=5.0):
        """
        Block until there are any subscribers to the given topic.

        @type topic: string
        @param topic: The topic to publish on.

        @type timeout: float
        @param timeout: How many seconds should be the maximum blocked time.
        """
        pub = ProxyPublisher._topics.get(topic)
        if pub is None:
            Logger.error("Publisher %s not yet registered, need to add it first!" % topic)
            return False

        tmr = Timer(.5, ProxyPublisher._print_wait_warning, [topic])
        tmr.start()
        available = ProxyPublisher._wait_for_subscribers(pub['publisher'], timeout)
        warning_sent = False
        try:
            tmr.cancel()
        except Exception:  # pylint: disable=W0703
            # already printed the warning
            warning_sent = True

        # Problem here
        if not available:
            Logger.error("Waiting for subscribers on %s timed out!" % topic)
            return False

        if warning_sent:
            Logger.info("Finally found subscriber on %s..." % (topic))

        return True

    @classmethod
    def _print_wait_warning(cls, topic):
        Logger.warning("Waiting for subscribers on %s..." % (topic))

    @classmethod
    def _wait_for_subscribers(cls, pub, timeout=5.0):
        polling_rate = 0.01
        count_down = int(timeout / polling_rate) + 1
        rate = Event()
        for _ in range(count_down, 0, -1):
            if pub.get_subscription_count() > 0:
                del rate
                return True
            rate.wait(polling_rate)  # Use system time for polling, not ROS possibly sim_time
        del rate
        return False

    @classmethod
    def destroy_publisher(cls, pub, topic):
        """Handle publisher destruction from within the executor threads."""
        try:
            if ProxyPublisher._node.destroy_publisher(pub):
                Logger.localinfo(f'Destroyed the proxy publisher for {topic}!')
            else:
                Logger.localwarn(f'Some issue destroying the proxy publisher for {topic}!')
            del pub
        except Exception as exc:  # pylint: disable=W0703
            Logger.error("Something went wrong destroying proxy publisher"
                         f" for {topic}!\n  {type(exc)} - {str(exc)}")
