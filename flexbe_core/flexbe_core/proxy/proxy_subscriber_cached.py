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
A proxy for subscribing topics that caches and buffers received messages.

Provides a single point for comminications for all states in behavior
"""

from functools import partial
from collections import defaultdict

from flexbe_core.logger import Logger
from flexbe_core.proxy.qos import QOS_DEFAULT


class ProxySubscriberCached:
    """A proxy for subscribing topics that caches and buffers received messages."""

    _node = None
    _topics = {}
    _persistant_topics = []

    @staticmethod
    def initialize(node):
        """Initialize ROS setup for proxy subscriber."""
        ProxySubscriberCached._node = node
        Logger.initialize(node)

    @staticmethod
    def shutdown():
        """Shut down this proxy by unregistering all subscribers."""
        try:
            print(f"Shutdown proxy subscriber with {len(ProxySubscriberCached._topics)} topics ...")
            for topic, topic_dict in ProxySubscriberCached._topics.items():
                try:
                    ProxySubscriberCached._topics[topic] = None
                    topic_dict['callbacks'] = {}
                    ProxySubscriberCached._node.destroy_subscription(topic_dict['subscription'])
                except Exception as exc:  # pylint: disable=W0703
                    Logger.error(f"Something went wrong during shutdown of proxy subscriber for "
                                 f"{topic}!\n{type(exc)} - {exc}")

            print("Shutdown proxy subscriber  ...")
            ProxySubscriberCached._topics.clear()
            ProxySubscriberCached._persistant_topics.clear()
            ProxySubscriberCached._node = None

        except Exception as exc:  # pylint: disable=W0703
            print(f'Something went wrong during shutdown of proxy subscriber !\n{str(exc)}')

    def __init__(self, topics=None, qos=None, inst_id=-1):
        """
        Initialize the proxy with optionally a given set of topics.

        @type topics: dictionary string - message_class
        @param topics: A dictionary containing a collection of topic - message type pairs.

        @type inst_id: int
        @param inst_id: identifier of instance creating subscription
        """
        if topics is not None:
            for topic, msg_type in topics.items():
                self.subscribe(topic, msg_type, qos=qos, inst_id=inst_id)

    @classmethod
    def subscribe(cls, topic, msg_type, callback=None, buffered=False, qos=None, inst_id=-1):  # pylint: disable=R0913
        """
        Add a new subscriber to the proxy.

        @type topic: string
        @param topic: The topic to subscribe.

        @type msg_type: a message class
        @param msg_type: The type of messages of this topic.

        @type callback: function
        @param callback: A function to be called when receiving messages.

        @type buffered: boolean
        @param buffered: True if all messages should be buffered,
                         False if only the last message should be cached.

        @type inst_id: int
        @param inst_id: identifier of instance creating subscription
        """
        if topic not in ProxySubscriberCached._topics:
            qos = qos or QOS_DEFAULT
            sub = ProxySubscriberCached._node.create_subscription(msg_type, topic,
                                                                  partial(cls._callback, topic=topic), qos)

            ProxySubscriberCached._topics[topic] = {'subscription': sub,
                                                    'last_msg': None,
                                                    'buffered': buffered,
                                                    'msg_queue': [],
                                                    'callbacks': defaultdict(None),
                                                    'subscribers': []}
            ProxySubscriberCached._topics[topic]['subscribers'].append(inst_id)
            Logger.localinfo(f"Created subscription for {topic} with message type {msg_type.__name__}!")

        else:
            if msg_type is not ProxySubscriberCached._topics[topic]['subscription'].msg_type:
                # Change in required msg_type for topic name  - update subscription with new type
                if msg_type.__name__ == ProxySubscriberCached._topics[topic]['subscription'].msg_type.__name__:
                    # Same message type name, so likely due to reloading Python module on behavior change
                    # Since we don't throw TypeErrors based on isinstance, and count on Python's duck typing
                    # for callbacks, we will ignore on FlexBE side for subscribers
                    if inst_id not in ProxySubscriberCached._topics[topic]['subscribers']:
                        Logger.localinfo(f"Add subscriber to existing subscription for {topic}"
                                         " - keep existing subscriber! ("
                                         f"{len(ProxySubscriberCached._topics[topic]['subscribers'])})")
                        ProxySubscriberCached._topics[topic]['subscribers'].append(inst_id)
                    else:
                        Logger.localinfo(f"Existing subscription for {topic} with same message type name"
                                         " - keep existing subscriber! "
                                         f"({len(ProxySubscriberCached._topics[topic]['subscribers'])})")
                else:
                    Logger.info(f"Mis-matched msg_types ({msg_type.__name__} vs. "
                                f"{ProxySubscriberCached._topics[topic]['subscription'].msg_type.__name__})"
                                f" for {topic} subscription (possibly due to reload of behavior)!")
                    raise TypeError("Trying to replace existing subscription with different msg type")
            else:
                if inst_id not in ProxySubscriberCached._topics[topic]['subscribers']:
                    Logger.localinfo(f"Add subscriber to existing subscription for {topic}!  "
                                     f"({len(ProxySubscriberCached._topics[topic]['subscribers'])})")
                    ProxySubscriberCached._topics[topic]['subscribers'].append(inst_id)
                else:
                    Logger.localinfo(f"Existing subscription for {topic} with same message type "
                                     "- keep existing subscriber! "
                                     f"({len(ProxySubscriberCached._topics[topic]['subscribers'])})")

        # Register the local callback for topic message
        cls.set_callback(topic, callback, inst_id)

    @classmethod
    def _callback(cls, msg, topic):
        """
        Execute all callbacks when a message is received.

        @type topic: message
        @param topic: The latest message received on this topic.

        @type topic: string
        @param topic: The topic to which this callback belongs.
        """
        if topic not in ProxySubscriberCached._topics:
            Logger.localinfo(f"-- invalid topic={topic} for callback!")
            return

        ProxySubscriberCached._topics[topic]['last_msg'] = msg
        if ProxySubscriberCached._topics[topic]['buffered']:
            ProxySubscriberCached._topics[topic]['msg_queue'].append(msg)

        for inst_id, callback in ProxySubscriberCached._topics[topic]['callbacks'].items():
            try:
                callback(msg)
            except Exception as exc:  # pylint: disable=W0703
                Logger.error(f"Exception in callback for {topic}: "
                             f"{callback.__module__}  {callback.__name__} @ {inst_id} \n {exc} ")

    @classmethod
    def set_callback(cls, topic, callback, inst_id=-1):
        """
        Add the given callback to the topic subscriber.

        @type topic: string
        @param topic: The topic to add the callback to.

        @type callback: function
        @param callback: The callback to be added.

        @type inst_id: int
        @param inst_id: identifier of instance creating subscription
        """
        if topic not in ProxySubscriberCached._topics:
            Logger.localinfo(f"-- invalid topic={topic} for set_callback @inst_id={inst_id}!")
            return

        if callback is not None:
            if inst_id not in ProxySubscriberCached._topics[topic]['callbacks']:
                ProxySubscriberCached._topics[topic]['callbacks'][inst_id] = callback
                Logger.localinfo(f"   Set local callback {callback.__name__} of "
                                 f"{len(ProxySubscriberCached._topics[topic]['callbacks'])} for {topic}!")
            else:
                Logger.localinfo("Update existing callback "
                                 f"{ProxySubscriberCached._topics[topic]['callbacks'][inst_id].__name__} with "
                                 f"{callback.__name__} of {len(ProxySubscriberCached._topics[topic]['callbacks'])}"
                                 f" for {topic}!")
                ProxySubscriberCached._topics[topic]['callbacks'][inst_id] = callback

    @classmethod
    def enable_buffer(cls, topic):
        """
        Enable the buffer on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxySubscriberCached._topics[topic]['buffered'] = True

    @classmethod
    def disable_buffer(cls, topic):
        """
        Disable the buffer on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxySubscriberCached._topics[topic]['buffered'] = False
        ProxySubscriberCached._topics[topic]['msg_queue'] = []

    @classmethod
    def is_available(cls, topic):
        """
        Check if the subscriber on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxySubscriberCached._topics

    @classmethod
    def get_last_msg(cls, topic):
        """
        Return the latest cached message of the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxySubscriberCached._topics[topic]['last_msg']

    @classmethod
    def get_from_buffer(cls, topic):
        """
        Pop the oldest buffered message of the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        if not ProxySubscriberCached._topics[topic]['buffered']:
            Logger.warning('Attempted to access buffer of non-buffered topic!')
            return None
        if len(ProxySubscriberCached._topics[topic]['msg_queue']) == 0:
            return None
        msg = ProxySubscriberCached._topics[topic]['msg_queue'][0]
        ProxySubscriberCached._topics[topic]['msg_queue'] = ProxySubscriberCached._topics[topic]['msg_queue'][1:]
        return msg

    @classmethod
    def has_msg(cls, topic):
        """
        Determine if the given topic has a message in its cache.

        @type topic: string
        @param topic: The topic of interest.
        """
        if cls.is_available(topic):
            return ProxySubscriberCached._topics[topic]['last_msg'] is not None
        return False

    @classmethod
    def has_buffered(cls, topic):
        """
        Determine if the given topic has any messages in its buffer.

        @type topic: string
        @param topic: The topic of interest.
        """
        return len(ProxySubscriberCached._topics[topic]['msg_queue']) > 0

    @classmethod
    def remove_last_msg(cls, topic, clear_buffer=False):
        """
        Remove the cached message of the given topic and optionally clears its buffer.

        @type topic: string
        @param topic: The topic of interest.

        @type topic: boolean
        @param topic: Set to true if the buffer of the given topic should be cleared as well.
        """
        if topic in ProxySubscriberCached._persistant_topics:
            return
        ProxySubscriberCached._topics[topic]['last_msg'] = None
        if clear_buffer:
            ProxySubscriberCached._topics[topic]['msg_queue'] = []

    @classmethod
    def make_persistant(cls, topic):
        """
        Make the given topic persistant which means messages can no longer be removed.

        Remove_last_msg will have no effect, only overwritten by a new message.

        @type topic: string
        @param topic: The topic of interest.
        """
        if topic not in ProxySubscriberCached._persistant_topics:
            ProxySubscriberCached._persistant_topics.append(topic)

    @classmethod
    def unsubscribe_topic(cls, topic, inst_id=-1):
        """
        Remove the given topic from the list of subscribed topics.

        @type topic: string
        @param topic: The topic of interest.

        @type inst_id: int
        @param inst_id: identifier of instance creating subscription
        """
        if topic in ProxySubscriberCached._topics:

            try:
                topic_dict = ProxySubscriberCached._topics[topic]
                if inst_id in topic_dict['subscribers']:
                    topic_dict['subscribers'].remove(inst_id)
                    Logger.localinfo(f"Unsubscribed {topic} from proxy! "
                                     f"({len(topic_dict['subscribers'])} remaining)")

                if inst_id in topic_dict['callbacks']:
                    topic_dict['callbacks'].pop(inst_id)
                    Logger.localinfo(f"Removed callback from proxy subscription for {topic} "
                                     f"from proxy! ({len(topic_dict['callbacks'])} remaining)")

                remaining_subscribers = len(topic_dict['subscribers'])
                remaining_callbacks = len(topic_dict['callbacks'])
                if remaining_subscribers == 0:
                    assert remaining_callbacks == 0, "Must have at least one subscriber tracked for every callback!"
                    sub = topic_dict['subscription']
                    ProxySubscriberCached._topics.pop(topic)
                    ProxySubscriberCached._node.executor.create_task(ProxySubscriberCached.destroy_subscription,
                                                                     sub, topic)

            except Exception as exc:  # pylint: disable=W0703
                Logger.error(f'Something went wrong unsubscribing {topic} of proxy subscriber!\n%s', str(exc))

    @classmethod
    def destroy_subscription(cls, sub, topic):
        """Handle subscription destruction from within the executor threads."""
        try:
            if ProxySubscriberCached._node.destroy_subscription(sub):
                Logger.localinfo(f'Destroyed the proxy subscription for {topic} ({id(sub)})!')
            else:
                Logger.localwarn(f'Some issue destroying the proxy subscription for {topic}!')
            del sub
        except Exception as exc:  # pylint: disable=W0703
            Logger.error("Something went wrong destroying subscription"
                         f" for {topic}!\n  {type(exc)} - {str(exc)}")
