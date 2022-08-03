from functools import partial
from rclpy.duration import Duration
from rclpy.action import ActionClient
from threading import Timer

from flexbe_core.logger import Logger


#TODO implement action client
class ProxyActionClient(object):
    """
    A proxy for calling actions.
    """
    _node = None
    _clients = {}
    _has_active_goal = {}
    _current_goal = {}
    _cancel_current_goal = {}

    _result = {}
    _feedback = {}

    @staticmethod
    def _initialize(node):
        ProxyActionClient._node = node
        Logger.initialize(node)

    def __init__(self, topics={}, wait_duration=10):
        """
        Initializes the proxy with optionally a given set of clients.

        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for each client in the
            given set to become available (if it is not already available).
        """
        for topic, action_type in topics.items():
            self.setupClient(topic, action_type, wait_duration)

    def setupClient(self, topic, action_type, wait_duration=10):
        """
        Tries to set up an action client for calling it later.

        @type topic: string
        @param topic: The topic of the action to call.

        @type action_type: action type
        @param action_type: The type of Action for this action client.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        if topic not in ProxyActionClient._clients:
            ProxyActionClient._clients[topic] = ActionClient(ProxyActionClient._node, action_type, topic)
            self._check_topic_available(topic, wait_duration)
        else:
            if action_type is not ProxyActionClient._clients[topic]._action_type:
                if action_type.__name__ == ProxyActionClient._clients[topic]._action_type.__name__:
                    Logger.localinfo(f'Existing action client for {topic} with same action type name, but different instance - recreate  client!')

                    # destroy() causes a crash under Humble
                    #ProxyActionClient._clients[topic].destroy()

                    ProxyActionClient._clients[topic] = ActionClient(ProxyActionClient._node, action_type, topic)
                    self._check_topic_available(topic, wait_duration)
                else:
                    raise TypeError("Trying to replace existing action client with different action type")


    def send_goal(self, topic, goal):
        """
        Performs an action call on the given topic.

        @type topic: string
        @param topic: The topic to call.

        @type goal: action goal
        @param goal: The request to send to the action server.
        """
        if not self._check_topic_available(topic):
            raise ValueError('Cannot send goal for action client %s: Topic not available.' % topic)
        # reset previous results
        ProxyActionClient._result[topic] = None
        ProxyActionClient._feedback[topic] = None
        ProxyActionClient._cancel_current_goal[topic] = False
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = None

        if not isinstance(goal, ProxyActionClient._clients[topic]._action_type.Goal):
            if goal.__class__.__name__ == ProxyActionClient._clients[topic]._action_type.Goal.__name__:
                # This is the case if the same class is imported multiple times
                # To avoid rclpy TypeErrors, we will automatically convert to the base type
                # used in the original service/publisher clients
                new_goal = ProxyActionClient._clients[topic]._action_type.Goal()
                Logger.localinfo(f"  converting goal {str(type(new_goal))} vs. {str(type(goal))}")
                assert new_goal.__slots__ == goal.__slots__, f"Message attributes for {topic} do not match!"
                for attr in goal.__slots__:
                    setattr(new_goal, attr, getattr(goal, attr))
            else:
                raise TypeError(f"Invalid goal type {goal.__class__.__name__} (vs. {ProxyActionClient._clients[topic]._action_type.Goal.__name__}) for topic {topic}")
        else:
            # Same class definition instance as stored
            new_goal = goal

        # send goal
        ProxyActionClient._clients[topic].wait_for_server()
        future = ProxyActionClient._clients[topic].send_goal_async(
            new_goal,
            feedback_callback=lambda f: self._feedback_callback(topic, f)
        )

        future.add_done_callback(partial(self._done_callback, topic=topic))

    def _done_callback(self, future, topic):
        ProxyActionClient._current_goal[topic] = future
        result = future.result().get_result_async()
        result.add_done_callback(partial(self._result_callback, topic=topic))

    def _result_callback(self, future, topic):
        result = future.result().result
        ProxyActionClient._result[topic] = result
        ProxyActionClient._has_active_goal[topic] = False

    def _feedback_callback(self, topic, feedback):
        ProxyActionClient._feedback[topic] = feedback

    def is_available(self, topic):
        """
        Checks if the client on the given action topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return self._check_topic_available(topic)

    def has_result(self, topic):
        """
        Checks if the given action call already has a result.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result.get(topic) is not None

    def get_result(self, topic):
        """
        Returns the result message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result.get(topic)

    def remove_result(self, topic):
        """
        Removes the latest result message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxyActionClient._result[topic] = None

    def has_feedback(self, topic):
        """
        Checks if the given action call has any feedback.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback.get(topic) is not None

    def get_feedback(self, topic):
        """
        Returns the latest feedback message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback.get(topic)

    def remove_feedback(self, topic):
        """
        Removes the latest feedback message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxyActionClient._feedback[topic] = None

    def get_state(self, topic):
        """
        Determines the current actionlib state of the given action topic.
        A list of possible states is defined in actionlib_msgs/GoalStatus.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._clients[topic].get_state()

    def is_active(self, topic):
        """
        Determines if an action request is already being processed on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._has_active_goal.get(topic, False)

    def cancel(self, topic):
        """
        Cancels the current action call on the given action topic.

        @type topic: string
        @param topic: The topic of interest.
        """

        current_goal = ProxyActionClient._current_goal.get(topic)
        if current_goal is not None:
            current_goal.result().cancel_goal()

        ProxyActionClient._cancel_current_goal[topic] = True
        ProxyActionClient._current_goal[topic] = None

    def _check_topic_available(self, topic, wait_duration=1):
        """
        Checks whether a topic is available.

        @type topic: string
        @param topic: The topic of the action.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        client = ProxyActionClient._clients.get(topic)
        if client is None:
            Logger.logerr("Action client %s not yet registered, need to add it first!" % topic)
            return False
        t = Timer(.5, self._print_wait_warning, [topic])
        t.start()
        available = client.wait_for_server(wait_duration)

        warning_sent = False
        try:
            t.cancel()
        except Exception:
            # already printed the warning
            warning_sent = True

        if not available:
            Logger.logerr("Action client %s timed out!" % topic)
            return False
        else:
            if warning_sent:
                Logger.loginfo("Finally found action client %s..." % (topic))
        return True

    def _print_wait_warning(self, topic):
        Logger.logwarn("Waiting for action client %s..." % (topic))
