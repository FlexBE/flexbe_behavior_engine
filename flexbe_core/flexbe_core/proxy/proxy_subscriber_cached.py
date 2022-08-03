from functools import partial
from collections import defaultdict

from flexbe_core.logger import Logger
from flexbe_core.proxy.qos import QOS_DEFAULT


class ProxySubscriberCached(object):
    """
    A proxy for subscribing topics that caches and buffers received messages.
    """
    _node = None
    _topics = {}
    _persistant_topics = []

    def _initialize(node):
        ProxySubscriberCached._node = node
        Logger.initialize(node)

    def __init__(self, topics={}, qos=None, id=-1):
        """
        Initializes the proxy with optionally a given set of topics.

        @type topics: dictionary string - message_class
        @param topics: A dictionary containing a collection of topic - message type pairs.

        @type id: int
        @param id: identifier of instance creating subscription
        """
        for topic, msg_type in topics.items():
            self.subscribe(topic, msg_type, qos=qos, id=id)

    def subscribe(self, topic, msg_type, callback=None, buffered=False, qos=None, id=-1):
        """
        Adds a new subscriber to the proxy.

        @type topic: string
        @param topic: The topic to subscribe.

        @type msg_type: a message class
        @param msg_type: The type of messages of this topic.

        @type callback: function
        @param callback: A function to be called when receiving messages.

        @type buffered: boolean
        @param buffered: True if all messages should be buffered, False if only the last message should be cached.

        @type id: int
        @param id: identifier of instance creating subscription
        """
        if topic not in ProxySubscriberCached._topics:
            qos = qos or QOS_DEFAULT
            sub = ProxySubscriberCached._node.create_subscription(msg_type, topic,
                                                                  partial(self._callback, topic=topic), qos)

            ProxySubscriberCached._topics[topic] = {'subscriber': sub,
                                                    'last_msg': None,
                                                    'buffered': buffered,
                                                    'msg_queue': [],
                                                    'callbacks': defaultdict(None),
                                                    'subscribers': []}
            ProxySubscriberCached._topics[topic]['subscribers'].append(id)
            Logger.localinfo(f"Created subscription for {topic} with message type {msg_type.__name__}!")

        else:
            if msg_type is not ProxySubscriberCached._topics[topic]['subscriber'].msg_type:
                # Change in required msg_type for topic name  - update subscription with new type
                if msg_type.__name__ == ProxySubscriberCached._topics[topic]['subscriber'].msg_type.__name__:
                    # Same message type name, so likely due to reloading Python module on behavior change
                    # Since we don't throw TypeErrors based on isinstance, and count on Python's duck typing
                    # for callbacks, we will ignore on FlexBE side for subscribers
                    if id not in ProxySubscriberCached._topics[topic]['subscribers']:
                        Logger.localinfo(f"Add subscriber to existing subscription for {topic} - keep existing subscriber! ({len(ProxySubscriberCached._topics[topic]['subscribers'])})")
                        ProxySubscriberCached._topics[topic]['subscribers'].append(id)
                    else:
                        Logger.localinfo(f"Existing subscription for {topic} with same message type name - keep existing subscriber! ({len(ProxySubscriberCached._topics[topic]['subscribers'])})")

                else:
                    Logger.info(f"Mis-matched msg_types ({msg_type.__name__} vs. " + \
                                f"{ProxySubscriberCached._topics[topic]['subscriber'].msg_type.__name__})" + \
                                f" for {topic} subscription (possibly due to reload of behavior)!")
                    raise TypeError("Trying to replace existing subscription with different msg type")
            else:
                if id not in ProxySubscriberCached._topics[topic]['subscribers']:
                    Logger.localinfo(f"Add subscriber to existing subscription for {topic}!  ({len(ProxySubscriberCached._topics[topic]['subscribers'])})")
                    ProxySubscriberCached._topics[topic]['subscribers'].append(id)
                else:
                    Logger.localinfo(f"Existing subscription for {topic} with same message type - keep existing subscriber! ({len(ProxySubscriberCached._topics[topic]['subscribers'])})")

        # Register the local callback for topic message
        self.set_callback(topic, callback, id)

    def _callback(self, msg, topic):
        """
        Standard callback that is executed when a message is received.

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

        #if len(ProxySubscriberCached._topics[topic]['callbacks']) > 0 and "heartbeat" not in topic:
        #        Logger.localinfo(f"-- process {len(ProxySubscriberCached._topics[topic]['callbacks'])} local callbacks for {topic} ...")

        for id, callback in ProxySubscriberCached._topics[topic]['callbacks'].items():
            try:
                #if "heartbeat" not in topic:
                #    Logger.localinfo(f"  -- process callback {callback.__name__} @ {id} for {topic}")
                callback(msg)
                #if "heartbeat" not in topic:
                #    Logger.localinfo(f"  -- processed callback {callback.__name__} @ {id} for {topic}")
            except Exception as e:
                Logger.error(f"Exception in callback for {topic}: {callback.__module__}  {callback.__name__} @ {id} \n {e} ")

        #if len(ProxySubscriberCached._topics[topic]['callbacks']) > 0 and "heartbeat" not in topic:
        #        Logger.localinfo(f"-- processed {len(ProxySubscriberCached._topics[topic]['callbacks'])} local callbacks for {topic} ...")


    def set_callback(self, topic, callback, id=-1):
        """
        Adds the given callback to the topic subscriber.

        @type topic: string
        @param topic: The topic to add the callback to.

        @type callback: function
        @param callback: The callback to be added.

        @type id: int
        @param id: identifier of instance creating subscription
        """
        if topic not in ProxySubscriberCached._topics:
            Logger.localinfo(f"-- invalid topic={topic} for set_callback @id={id}!")
            return

        if callback is not None:
            if id not in ProxySubscriberCached._topics[topic]['callbacks']:
                ProxySubscriberCached._topics[topic]['callbacks'][id] = callback
                Logger.localinfo(f"   Set local callback {callback.__name__} of {len(ProxySubscriberCached._topics[topic]['callbacks'])} for {topic}!")
            else:
                Logger.localinfo(f"Update existing callback {ProxySubscriberCached._topics[topic]['callbacks'][id].__name__} with " +\
                                 f"{callback.__name__} of {len(ProxySubscriberCached._topics[topic]['callbacks'])} for {topic}!")
                ProxySubscriberCached._topics[topic]['callbacks'][id] = callback

    def enable_buffer(self, topic):
        """
        Enables the buffer on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxySubscriberCached._topics[topic]['buffered'] = True

    def disable_buffer(self, topic):
        """
        Disables the buffer on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxySubscriberCached._topics[topic]['buffered'] = False
        ProxySubscriberCached._topics[topic]['msg_queue'] = []

    def is_available(self, topic):
        """
        Checks if the subscriber on the given topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxySubscriberCached._topics

    def get_last_msg(self, topic):
        """
        Returns the latest cached message of the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxySubscriberCached._topics[topic]['last_msg']

    def get_from_buffer(self, topic):
        """
        Pops the oldest buffered message of the given topic.

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

    def has_msg(self, topic):
        """
        Determines if the given topic has a message in its cache.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxySubscriberCached._topics[topic]['last_msg'] is not None

    def has_buffered(self, topic):
        """
        Determines if the given topic has any messages in its buffer.

        @type topic: string
        @param topic: The topic of interest.
        """
        return len(ProxySubscriberCached._topics[topic]['msg_queue']) > 0

    def remove_last_msg(self, topic, clear_buffer=False):
        """
        Removes the cached message of the given topic and optionally clears its buffer.

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

    def make_persistant(self, topic):
        """
        Makes the given topic persistant which means messages can no longer be removed
        (remove_last_msg will have no effect), only overwritten by a new message.

        @type topic: string
        @param topic: The topic of interest.
        """
        if topic not in ProxySubscriberCached._persistant_topics:
            ProxySubscriberCached._persistant_topics.append(topic)

    # def has_topic(self, topic):
    #     """
    #     Determines if the given topic is already subscribed.
    #
    #     @type topic: string
    #     @param topic: The topic of interest.
    #     """
    #     Logger.warning('Deprecated (ProxySubscriberCached): use "is_available(topic)" instead of "has_topic(topic)".')
    #     return self.is_available(topic)

    def unsubscribe_topic(self, topic, id=-1):
        """
        Removes the given topic from the list of subscribed topics.

        @type topic: string
        @param topic: The topic of interest.

        @type id: int
        @param id: identifier of instance creating subscription
        """
        if topic in ProxySubscriberCached._topics:

            try:
                if id in ProxySubscriberCached._topics[topic]['subscribers']:
                    ProxySubscriberCached._topics[topic]['subscribers'].remove(id)
                    Logger.localinfo(f"Unsubscribed {topic} from proxy! ({len(ProxySubscriberCached._topics[topic]['subscribers'])} remaining)" )

                if id in ProxySubscriberCached._topics[topic]['callbacks']:
                    ProxySubscriberCached._topics[topic]['callbacks'].pop(id)
                    Logger.localinfo(f"Removed callback from proxy subscription for {topic} from proxy! ({len(ProxySubscriberCached._topics[topic]['callbacks'])} remaining)")

                if len(ProxySubscriberCached._topics[topic]['subscribers']) == 0:
                    Logger.localinfo(f'Remove proxy subscriber with no customers for {topic} ...')
                    #Crashes Humble - ProxySubscriberCached._topics[topic]['subscriber'].destroy()
                    ProxySubscriberCached._topics.pop(topic)
            except Exception as e:
                Logger.error(f'Something went wrong unsubscribing {topic} of proxy subscriber!\n%s', str(e))

    def shutdown(self):
        """ Shuts this proxy down by unregistering all subscribers. """
        try:
            Logger.localinfo(f"Shutdown proxy subscriber {id(self)} with {len(ProxySubscriberCached._topics)} topics ..." )
            for topic in ProxySubscriberCached._topics:
                try:
                    ProxySubscriberCached._topics[topic]['callbacks'] = {}
                    #Crashes Humble - ProxySubscriberCached._topics[topic]['subscriber'].destroy()
                except Exception as e:
                    Logger.error(f"Something went wrong during shutdown of proxy subscriber {id(self)} for {topic}!\n%s", str(e))
        except Exception as e:
            Logger.error(f'Something went wrong during shutdown of proxy subscriber {id(self)} !\n%s', str(e))
        ProxySubscriberCached._topics.clear()
