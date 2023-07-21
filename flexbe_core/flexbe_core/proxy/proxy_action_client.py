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


"""A proxy for calling actions provides a single point for all state action interfaces."""
from functools import partial
from threading import Timer

from rclpy.action import ActionClient

from flexbe_core.logger import Logger


class ProxyActionClient:
    """A proxy for calling actions."""

    _node = None
    _clients = {}
    _has_active_goal = {}
    _current_goal = {}
    _cancel_current_goal = {}

    _result = {}
    _feedback = {}

    @staticmethod
    def initialize(node):
        """Initialize ROS setup for proxy action client."""
        ProxyActionClient._node = node
        Logger.initialize(node)

    @staticmethod
    def shutdown():
        """Shuts this proxy down by reseting all action clients."""
        try:
            print(f"Shutdown proxy action clients with {len(ProxyActionClient._clients)} topics ...")
            for topic, client in ProxyActionClient._clients.items():
                try:
                    ProxyActionClient._clients[topic] = None
                    ProxyActionClient._node.destroy_client(client)
                except Exception as exc:  # pylint: disable=W0703
                    Logger.error(f"Something went wrong during shutdown of proxy action client for {topic}!\n{str(exc)}")

            print("Shutdown proxy action clients  ...")
            ProxyActionClient._result.clear()
            ProxyActionClient._feedback.clear()
            ProxyActionClient._cancel_current_goal.clear()
            ProxyActionClient._has_active_goal.clear()
            ProxyActionClient._current_goal.clear()
            ProxyActionClient._node = None
        except Exception as exc:  # pylint: disable=W0703
            print(f'Something went wrong during shutdown of proxy action clients!\n{ str(exc)}')

    def __init__(self, topics=None, wait_duration=10):
        """
        Initialize the proxy with optionally a given set of clients.

        @type topics: dictionary string - message class
        @param topics: A dictionay containing a collection of topic - message type pairs.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for each client in the
            given set to become available (if it is not already available).
        """
        if topics is not None:
            for topic, action_type in topics.items():
                ProxyActionClient.setupClient(topic, action_type, wait_duration)

    @classmethod
    def setupClient(cls, topic, action_type, wait_duration=10):
        """
        Set up an action client for calling it later.

        @type topic: string
        @param topic: The topic of the action to call.

        @type action_type: action type
        @param action_type: The type of Action for this action client.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        if topic not in ProxyActionClient._clients:
            ProxyActionClient._clients[topic] = ActionClient(ProxyActionClient._node, action_type, topic)
            ProxyActionClient._check_topic_available(topic, wait_duration)

        else:
            if action_type is not ProxyActionClient._clients[topic]._action_type:
                if action_type.__name__ == ProxyActionClient._clients[topic]._action_type.__name__:
                    Logger.localinfo(f'Existing action client for {topic}'
                                     f' with same action type name, but different instance -  re-create  client!')

                    client = ProxyActionClient._clients[topic]
                    try:
                        client.destroy()
                    except Exception as exc:
                        Logger.localinfo(f'Exception destroying old client for {topic}'
                                         f'{type(exc)} - {exc}')

                    ProxyActionClient._clients[topic] = ActionClient(ProxyActionClient._node, action_type, topic)
                    ProxyActionClient._check_topic_available(topic, wait_duration)
                else:
                    raise TypeError("Trying to replace existing action client with different action type")

    @classmethod
    def send_goal(cls, topic, goal, wait_duration=0.0):
        """
        Call action on the given topic.

        @type topic: string
        @param topic: The topic to call.

        @type goal: action goal
        @param goal: The request to send to the action server.

        @type wait_duration: float seconds
        @param wait_duration: How long to wait for server
        """
        if not ProxyActionClient._check_topic_available(topic, wait_duration=wait_duration):
            raise ValueError(f'Cannot send goal for action client {topic}: Topic not available.')
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
                raise TypeError(f"Invalid goal type {goal.__class__.__name__}"
                                f" (vs. {ProxyActionClient._clients[topic]._action_type.Goal.__name__}) for topic {topic}")
        else:
            # Same class definition instance as stored
            new_goal = goal

        # send goal
        ProxyActionClient._clients[topic].wait_for_server()
        future = ProxyActionClient._clients[topic].send_goal_async(
            new_goal,
            feedback_callback=lambda f: ProxyActionClient._feedback_callback(topic, f)
        )

        future.add_done_callback(partial(ProxyActionClient._done_callback, topic=topic))

    @classmethod
    def _done_callback(cls, future, topic):
        ProxyActionClient._current_goal[topic] = future
        result = future.result().get_result_async()
        result.add_done_callback(partial(ProxyActionClient._result_callback, topic=topic))

    @classmethod
    def _result_callback(cls, future, topic):
        result = future.result().result
        ProxyActionClient._result[topic] = result
        ProxyActionClient._has_active_goal[topic] = False

    @classmethod
    def _feedback_callback(cls, topic, feedback):
        ProxyActionClient._feedback[topic] = feedback

    @classmethod
    def is_available(cls, topic):
        """
        Check if the client and server for the given action topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        client = ProxyActionClient._clients.get(topic)
        if client is None:
            Logger.logerr("Action client '%s' is not yet registered, need to add it first!" % topic)
            return False

        return client.server_is_ready()

    @classmethod
    def has_result(cls, topic):
        """
        Check if the given action call already has a result.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result.get(topic) is not None

    @classmethod
    def get_result(cls, topic):
        """
        Return the result message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result.get(topic)

    @classmethod
    def remove_result(cls, topic):
        """
        Remove the latest result message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxyActionClient._result[topic] = None

    @classmethod
    def has_feedback(cls, topic):
        """
        Check if the given action call has any feedback.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback.get(topic) is not None

    @classmethod
    def get_feedback(cls, topic):
        """
        Return the latest feedback message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback.get(topic)

    @classmethod
    def remove_feedback(cls, topic):
        """
        Remove the latest feedback message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxyActionClient._feedback[topic] = None

    @classmethod
    def get_state(cls, topic):
        """
        Determine the current actionlib state of the given action topic.

        A list of possible states is defined in actionlib_msgs/GoalStatus.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._clients[topic].get_state()

    @classmethod
    def is_active(cls, topic):
        """
        Determine if an action request is already being processed on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._has_active_goal.get(topic, False)

    @classmethod
    def cancel(cls, topic):
        """
        Cancel the current action call on the given action topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        current_goal = ProxyActionClient._current_goal.get(topic)
        if current_goal is not None:
            current_goal.result().cancel_goal()

        ProxyActionClient._cancel_current_goal[topic] = True
        ProxyActionClient._current_goal[topic] = None

    @classmethod
    def _check_topic_available(cls, topic, wait_duration=0.1):
        """
        Check whether a topic is available.

        @type topic: string
        @param topic: The topic of the action.

        @type wait_duration: int
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        client = ProxyActionClient._clients.get(topic)
        if client is None:
            Logger.logerr("Action client '%s' is not yet registered, need to add it first!" % topic)
            return False

        if wait_duration > 2.0:
            tmr = Timer(.5, ProxyActionClient._print_wait_warning, [topic])
            tmr.start()

        available = client.wait_for_server(wait_duration)

        warning_sent = False
        if wait_duration > 2.0:
            try:
                tmr.cancel()
            except Exception:  # pylint: disable=W0703
                # already printed the warning
                warning_sent = True

        if not available:
            Logger.logerr(f"Action client/server '{topic}' is not available - timed out after {wait_duration:.3f} seconds!")
            return False

        if warning_sent:
            Logger.loginfo(f"Finally found action client/server '{topic}'!")

        return True

    @classmethod
    def _print_wait_warning(cls, topic):
        Logger.logwarn(f"Waiting for action client/server for '{topic}'")
