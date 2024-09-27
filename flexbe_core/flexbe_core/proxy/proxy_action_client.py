# Copyright 2024 Philipp Schillinger, Team ViGIR, Christopher Newport University
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
from threading import Event, Lock, Timer

from action_msgs.msg import GoalStatus

from flexbe_core.logger import Logger

from rclpy.action import ActionClient
from rclpy.duration import Duration


class ProxyActionClient:
    """A proxy for calling actions."""

    _node = None
    _clients = {}
    _has_active_goal = {}
    _current_goal = {}

    _result = {}
    _result_status = {}
    _feedback = {}

    _client_sync_lock = Lock()

    _goal_status_dict = {
        0: 'UNKNOWN',
        1: 'ACCEPTED',
        2: 'EXECUTING',
        3: 'CANCELING',
        4: 'SUCCEEDED',
        5: 'CANCELED',
        6: 'ABORTED'
    }

    @staticmethod
    def initialize(node):
        """Initialize ROS setup for proxy action client."""
        ProxyActionClient._node = node
        Logger.initialize(node)

    @staticmethod
    def shutdown():
        """Shuts this proxy down by resetting all action clients."""
        try:
            print(f'Shutdown proxy action clients with {len(ProxyActionClient._clients)} topics ...')
            for topic, client_dict in ProxyActionClient._clients.items():
                try:
                    ProxyActionClient._clients[topic] = None
                    ProxyActionClient._node.destroy_client(client_dict['client'])
                except Exception as exc:  # pylint: disable=W0703
                    Logger.error(f"Something went wrong during shutdown of proxy action client for '{topic}'!\n{str(exc)}")

            ProxyActionClient._result.clear()
            ProxyActionClient._result_status.clear()
            ProxyActionClient._feedback.clear()
            ProxyActionClient._has_active_goal.clear()
            ProxyActionClient._current_goal.clear()
        except Exception as exc:  # pylint: disable=W0703
            print(f'Something went wrong during shutdown of proxy action clients!\n{ str(exc)}')

    def __init__(self, topics=None, wait_duration=1.0):
        """
        Initialize the proxy with an optionally given set of clients.

        @type topics: dictionary string - message class
        @param topics: A dictionary containing a collection of topic - message type pairs.

        @type wait_duration: float
        @param wait_duration: Defines how long to wait for each client in the
            given set to become available (if it is not already available).
        """
        if topics is not None:
            for topic, action_type in topics.items():
                ProxyActionClient.setup_client(topic, action_type, wait_duration)

    @classmethod
    def setupClient(cls, topic, action_type, wait_duration=1.0):
        """Set up proxy action client (Deprecated)."""
        Logger.localerr('Deprecated: Use ProxyActionClient.setup_client instead!')
        cls.setup_client(topic, action_type, wait_duration)

    @classmethod
    def setup_client(cls, topic, action_type, wait_duration=None):
        """
        Set up an action client for calling it later.

        @type topic: string
        @param topic: The topic of the action to call.

        @type action_type: action type
        @param action_type: The type of Action for this action client.

        @type wait_duration: float
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        with cls._client_sync_lock:
            if topic not in ProxyActionClient._clients:
                ProxyActionClient._clients[topic] = {'client': ActionClient(ProxyActionClient._node, action_type, topic),
                                                     'count': 1}

            else:
                if action_type is not ProxyActionClient._clients[topic]['client']._action_type:
                    if action_type.__name__ == ProxyActionClient._clients[topic]['client']._action_type.__name__:
                        if ProxyActionClient._clients[topic]['count'] == 1:
                            Logger.localinfo(f'Existing action client for {topic}'
                                             f' with same action type name, but different instance -  re-create  client!')
                        else:
                            Logger.localwarn(f"Existing action client for '{topic}' "
                                             f"with {ProxyActionClient._clients[topic]['count']} references\n"
                                             f'    with same action type name, but different instance\n'
                                             f'    just re-create client with 1 reference - but be warned!')

                        # Destroy the existing client in executor thread
                        client = ProxyActionClient._clients[topic]['client']
                        ProxyActionClient._node.executor.create_task(ProxyActionClient.destroy_client, client, topic)

                        ProxyActionClient._clients[topic] = {'client': ActionClient(ProxyActionClient._node,
                                                                                    action_type, topic),
                                                             'count': 1}
                    else:
                        raise TypeError('Trying to replace existing action client with different action type')
                else:
                    ProxyActionClient._clients[topic]['count'] = ProxyActionClient._clients[topic]['count'] + 1

        if isinstance(wait_duration, (float, int)):
            ProxyActionClient._check_topic_available(topic, wait_duration)

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

        client_dict = ProxyActionClient._clients.get(topic)
        if client_dict is None:
            raise ValueError(f'Cannot send goal for action client {topic}: Client is not initialized.')
        client = client_dict['client']

        # reset previous results
        ProxyActionClient._result[topic] = None
        ProxyActionClient._result_status[topic] = None
        ProxyActionClient._feedback[topic] = None
        ProxyActionClient._has_active_goal[topic] = True
        ProxyActionClient._current_goal[topic] = None

        if not isinstance(goal, client._action_type.Goal):
            if goal.__class__.__name__ == ProxyActionClient._clients[topic]._action_type.Goal.__name__:
                # This is the case if the same class is imported multiple times
                # To avoid rclpy TypeErrors, we will automatically convert to the base type
                # used in the original service/publisher clients
                new_goal = client._action_type.Goal()
                Logger.localinfo(f"  converting goal '{str(type(new_goal))}' vs. '{str(type(goal))}'")
                assert new_goal.__slots__ == goal.__slots__, f"Message attributes for '{topic}' do not match!"
                for attr in goal.__slots__:
                    setattr(new_goal, attr, getattr(goal, attr))
            else:
                raise TypeError(f"Invalid goal type '{goal.__class__.__name__}'"
                                f" (vs. '{ProxyActionClient._clients[topic]._action_type.Goal.__name__}') for topic '{topic}'")
        else:
            # Same class definition instance as stored
            new_goal = goal

        # send goal
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_UNKNOWN
        client.wait_for_server()
        future = client.send_goal_async(new_goal,
                                        feedback_callback=lambda f: ProxyActionClient._feedback_callback(topic, f)
                                        )
        future.add_done_callback(partial(ProxyActionClient._done_callback, topic=topic))

    @classmethod
    def _done_callback(cls, future, topic):
        ProxyActionClient._current_goal[topic] = future
        if future.result().accepted:
            result = future.result().get_result_async()
            ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ACCEPTED
            result.add_done_callback(partial(ProxyActionClient._result_callback, topic=topic))
        else:
            ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ABORTED
            ProxyActionClient._has_active_goal[topic] = False
            Logger.localinfo(f"Goal for '{topic}' ({future.result().goal_id.uuid}) was rejected!")

    @classmethod
    def _result_callback(cls, future, topic):
        result = future.result().result
        result_status = future.result().status
        ProxyActionClient._result[topic] = result
        ProxyActionClient._result_status[topic] = result_status
        ProxyActionClient._has_active_goal[topic] = False

    @classmethod
    def _feedback_callback(cls, topic, feedback):
        ProxyActionClient._feedback[topic] = feedback.feedback

    @classmethod
    def is_available(cls, topic):
        """
        Check if the client and server for the given action topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        client_dict = ProxyActionClient._clients.get(topic)
        if client_dict is None:
            Logger.logerr("Action client '%s' is not yet registered, need to add it first!" % topic)
            return False

        client = client_dict['client']
        if client is None:
            Logger.logerr("Action client '%s' is not yet initialized, need to add it first!" % topic)
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
    def get_result(cls, topic, clear=False):
        """
        Return the result message of the given action call.

        :param topic: The topic of interest.
        :type topic: str
        :param clear: Clear the prior response
        :type clear: bool
        """
        result = ProxyActionClient._result.get(topic)

        if clear and result is not None:
            ProxyActionClient._result[topic] = None

        return result

    @classmethod
    def remove_result(cls, topic):
        """
        Remove the latest results of the given action call.

        Typically called in on_enter before making sending new goal.
        @type topic: string
        @param topic: The topic of interest.
        """
        if topic not in ProxyActionClient._result:
            return

        if ProxyActionClient._has_active_goal[topic]:
            Logger.localwarn(f"Request to remove result of action '{topic}' with active goal!")

        # Clear old data
        ProxyActionClient._feedback[topic] = None
        ProxyActionClient._has_active_goal[topic] = False
        ProxyActionClient._result[topic] = None
        ProxyActionClient._result_status[topic] = None

    @classmethod
    def has_feedback(cls, topic):
        """
        Check if the given action call has any feedback.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback.get(topic) is not None

    @classmethod
    def get_feedback(cls, topic, clear=False):
        """
        Return the latest feedback message of the given action call.

        :param topic: The topic of interest.
        :type topic: str
        :param clear: Clear the prior response
        :type clear: bool
        """
        feedback = ProxyActionClient._feedback.get(topic)
        if clear:
            ProxyActionClient._feedback[topic] = None

        return feedback

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
        Determine the action status of the given action topic.

        A list of possible states is defined in action_msgs/GoalStatus.

        @type topic: string
        @param topic: The topic of interest.

        @deprecated: This method is deprecated and will be removed in a future release.
                Use the `get_status` method instead.
        """
        return ProxyActionClient._result_status.get(topic)

    @classmethod
    def get_status(cls, topic):
        """
        Determine the action server status of the given action topic.

        A list of possible states is defined in action_msgs/GoalStatus.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result_status.get(topic)

    @classmethod
    def get_status_string(cls, topic):
        """
        Return the action server status of the given action topic as a string.

        A list of possible states is defined in action_msgs/GoalStatus.

        @type topic: string
        @param topic: The topic of interest.
        """
        status_code = ProxyActionClient._result_status.get(topic)
        if status_code in cls._goal_status_dict:
            return cls._goal_status_dict[status_code]

        return 'Unknown Status'

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

        :param topic: The topic of interest.
        :type topic: str
        :param block: Whether to wait for the cancel response.
        :type block: bool
        """
        Logger.localinfo(f"Request to cancel '{topic}' ...")
        current_goal_future = ProxyActionClient._current_goal.get(topic)
        if current_goal_future is not None:
            try:
                current_goal_handle = current_goal_future.result()
                cancel_future = current_goal_handle.cancel_goal_async()
                ProxyActionClient._result_status[topic] = GoalStatus.STATUS_CANCELING

                # add callback to acknowledge completion of cancel_goal
                cancel_future.add_done_callback(partial(ProxyActionClient._cancel_callback, topic=topic))

            except Exception as exc:
                Logger.localinfo(f"  Error canceling '{topic}' : {exc}")
        else:
            Logger.localinfo(f"Failed to send cancel request for '{topic}' ...")

        ProxyActionClient._current_goal[topic] = None

    @classmethod
    def _cancel_callback(cls, future, topic):
        result = future.result()
        Logger.localinfo(f"   cancel result for '{topic}' : result={result.return_code}")
        ProxyActionClient._result_status[topic] = GoalStatus.STATUS_CANCELED
        ProxyActionClient._has_active_goal[topic] = False

    @classmethod
    def verify_action_status(cls, topic, wait_duration=0.1):
        """
        Verify action is in terminal status if topic is available.

        This is a blocking call with polling and should be used sparingly!

        @type topic: string
        @param topic: The topic of the action.

        @return (is_terminal, status code)
        """
        if topic not in ProxyActionClient._result_status:
            return None

        rate = Event()
        timeout = Duration(seconds=wait_duration)
        start = cls._node.get_clock().now()
        terminal_statuses = (GoalStatus.STATUS_ABORTED,
                             GoalStatus.STATUS_CANCELED,
                             GoalStatus.STATUS_SUCCEEDED)
        while cls._node.get_clock().now() - start < timeout:
            if ProxyActionClient._result_status[topic] in terminal_statuses:
                Logger.localinfo(f"Action '{topic}' returned terminal status "
                                 f"'{ProxyActionClient.get_status_string(topic)}' after "
                                 f'{(cls._node.get_clock().now() - start).nanoseconds * 1e-9:.6f} seconds')
                return True, ProxyActionClient._result_status[topic]
            rate.wait(0.002)
        return False, ProxyActionClient._result_status[topic]

    @classmethod
    def _check_topic_available(cls, topic, wait_duration=0.1):
        """
        Check whether a topic is available.

        @type topic: string
        @param topic: The topic of the action.

        @type wait_duration: float
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        client_dict = ProxyActionClient._clients.get(topic)
        if client_dict is None:
            Logger.logerr("Action client '%s' is not yet registered, need to add it first!" % topic)
            return False

        if wait_duration > 2.0:
            tmr = Timer(.5, ProxyActionClient._print_wait_warning, [topic])
            tmr.start()

        client = client_dict['client']
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

    @classmethod
    def remove_client(cls, topic):
        """
        Remove action client from proxy.

        @type topic: string
        @param topic: The topic to publish on.
        """
        client = None
        count = -1
        with cls._client_sync_lock:
            if topic in ProxyActionClient._clients:
                ProxyActionClient._clients[topic]['count'] = ProxyActionClient._clients[topic]['count'] - 1
                count = ProxyActionClient._clients[topic]['count']
                if count < 1:
                    client = ProxyActionClient._clients[topic]['client']
                    ProxyActionClient._clients.pop(topic)

                    if topic in ProxyActionClient._result:
                        ProxyActionClient._result.pop(topic)

                    if topic in ProxyActionClient._feedback:
                        ProxyActionClient._feedback.pop(topic)

        if client is not None:
            Logger.localdebug(f"Action client for '{topic}' has {count} references remaining.")
            ProxyActionClient._node.executor.create_task(ProxyActionClient.destroy_client, client, topic)
        else:
            Logger.localdebug(f"Publisher for '{topic}' remains with {count} references!")

    @classmethod
    def destroy_client(cls, client, topic):
        """Handle client destruction from within the executor threads."""
        try:
            del client
            Logger.localinfo(f'Destroyed the proxy action client for {topic}!')
        except Exception as exc:  # pylint: disable=W0703
            Logger.error('Something went wrong destroying proxy action client'
                         f" for '{topic}'!\n  {type(exc)} - {str(exc)}")
