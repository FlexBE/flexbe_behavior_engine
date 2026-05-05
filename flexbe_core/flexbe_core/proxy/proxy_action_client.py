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
from action_msgs.srv import CancelGoal

from flexbe_core.core.exceptions import ProxyAvailabilityError, ProxyTypeError, ShutdownError
from flexbe_core.logger import Logger

from rclpy.action import ActionClient
from rclpy.duration import Duration


class ProxyActionClient:
    """A proxy for calling actions.

    This proxy shares one ROS 2 ``ActionClient`` per topic, but intentionally tracks
    only one active goal per topic at a time. Callers must cancel or finish the
    current goal before sending another goal on the same topic.
    """

    _node = None
    _clients = {}
    _has_active_goal = {}
    _current_goal = {}

    _result = {}
    _result_status = {}
    _feedback = {}
    _is_shutting_down = False
    _client_generation_counter = 0

    _client_sync_lock = Lock()

    _goal_status_dict = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }

    @staticmethod
    def initialize(node):
        """Initialize ROS setup for proxy action client."""
        ProxyActionClient._is_shutting_down = False
        ProxyActionClient._node = node
        Logger.initialize(node)

    @classmethod
    def _next_client_generation(cls):
        """Return a new generation identifier for a topic client incarnation."""
        cls._client_generation_counter += 1
        return cls._client_generation_counter

    @classmethod
    def _get_client_generation(cls, topic):
        """Return the generation id for the active client on the topic."""
        client_dict = cls._clients.get(topic)
        if client_dict is None:
            raise ProxyAvailabilityError(f"No action client registered for '{topic}'.")
        return client_dict.get('generation')

    @classmethod
    def _matches_client_generation(cls, topic, client_generation):
        """Return True if the callback still belongs to the active client incarnation."""
        client_dict = cls._clients.get(topic)
        return client_dict is not None and client_dict.get('generation') == client_generation

    @staticmethod
    def shutdown():
        """Shuts this proxy down by resetting all action clients."""
        with ProxyActionClient._client_sync_lock:
            ProxyActionClient._is_shutting_down = True
            print(f'Shutdown proxy action clients with {len(ProxyActionClient._clients)} topics ...')
            for topic, client_dict in list(ProxyActionClient._clients.items()):
                try:
                    ProxyActionClient._shutdown_topic_client(topic, client_dict)
                except (ProxyTypeError, ShutdownError) as exc:
                    print(f"Something went wrong during shutdown of proxy action client for '{topic}'!\n{str(exc)}", flush=True)

            ProxyActionClient._clients.clear()
            ProxyActionClient._result.clear()
            ProxyActionClient._result_status.clear()
            ProxyActionClient._feedback.clear()
            ProxyActionClient._has_active_goal.clear()
            ProxyActionClient._current_goal.clear()

    @staticmethod
    def _shutdown_topic_client(topic, client_dict):
        """Shutdown one topic client and raise typed errors on invalid state."""
        if isinstance(client_dict, dict):
            client = client_dict.get('client')
        elif client_dict is None:
            client = None
        else:
            raise ProxyTypeError(f"Expected client dictionary for '{topic}', got '{type(client_dict).__name__}'.")

        ProxyActionClient._clients[topic] = None
        if client is None:
            return
        if ProxyActionClient._node is None:
            raise ShutdownError(f"Proxy node is not initialized while shutting down '{topic}'.")
        try:
            ProxyActionClient._node.destroy_client(client)
        except RuntimeError as exc:
            raise ShutdownError(f"Failed to destroy client for '{topic}': {exc}") from exc

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
                generation = cls._next_client_generation()
                ProxyActionClient._clients[topic] = {'client': ActionClient(ProxyActionClient._node, action_type, topic),
                                                     'generation': generation,
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

                        generation = cls._next_client_generation()
                        ProxyActionClient._clients[topic] = {'client': ActionClient(ProxyActionClient._node,
                                                                                    action_type, topic),
                                                             'generation': generation,
                                                             'count': 1}
                        ProxyActionClient._result.pop(topic, None)
                        ProxyActionClient._result_status.pop(topic, None)
                        ProxyActionClient._feedback.pop(topic, None)
                        ProxyActionClient._has_active_goal.pop(topic, None)
                        ProxyActionClient._current_goal.pop(topic, None)
                    else:
                        raise ProxyTypeError('Trying to replace existing action client with different action type')
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
        @param wait_duration: How long to wait for server readiness before raising.
            The default of 0.0 is fail-fast: only dispatch if the server is already ready.
        """
        if ProxyActionClient._has_active_goal.get(topic, False):
            raise ProxyAvailabilityError(f'Cannot send goal for action client {topic}: A goal is already active. '
                                         'Cancel it before sending a new goal.')
        if not ProxyActionClient._check_topic_available(topic, wait_duration=wait_duration):
            raise ProxyAvailabilityError(f'Cannot send goal for action client {topic}: Topic not available.')

        client_dict = ProxyActionClient._clients.get(topic)
        if client_dict is None:
            raise ProxyAvailabilityError(f'Cannot send goal for action client {topic}: Client is not initialized.')
        client = client_dict['client']
        client_generation = client_dict['generation']

        if not isinstance(goal, client._action_type.Goal):
            if goal.__class__.__name__ == client._action_type.Goal.__name__:
                # This is the case if the same class is imported multiple times
                # To avoid rclpy TypeErrors, we will automatically convert to the base type
                # used in the original service/publisher clients
                new_goal = client._action_type.Goal()
                Logger.localinfo(f"  converting goal '{str(type(new_goal))}' vs. '{str(type(goal))}'")
                assert new_goal.__slots__ == goal.__slots__, f"Message attributes for '{topic}' do not match!"
                for attr in goal.__slots__:
                    setattr(new_goal, attr, getattr(goal, attr))
            else:
                raise ProxyTypeError(f"Invalid goal type '{goal.__class__.__name__}'"
                                     f" (vs. '{client._action_type.Goal.__name__}') for topic '{topic}'")
        else:
            # Same class definition instance as stored
            new_goal = goal

        previous_result = ProxyActionClient._result.get(topic)
        previous_feedback = ProxyActionClient._feedback.get(topic)
        previous_status = ProxyActionClient._result_status.get(topic)
        previous_has_active_goal = ProxyActionClient._has_active_goal.get(topic, False)
        previous_goal = ProxyActionClient._current_goal.get(topic)

        try:
            future = client.send_goal_async(new_goal,
                                            feedback_callback=lambda f:
                                            ProxyActionClient._feedback_callback(topic, f, client_generation)
                                            )
            # Set the active-goal bookkeeping before registering callbacks because
            # rclpy may invoke done callbacks immediately for already-complete futures.
            ProxyActionClient._result[topic] = None
            ProxyActionClient._result_status[topic] = GoalStatus.STATUS_UNKNOWN
            ProxyActionClient._feedback[topic] = None
            ProxyActionClient._has_active_goal[topic] = True
            ProxyActionClient._current_goal[topic] = future
            future.add_done_callback(partial(ProxyActionClient._done_callback,
                                             topic=topic,
                                             client_generation=client_generation))
        except Exception:
            ProxyActionClient._result[topic] = previous_result
            ProxyActionClient._feedback[topic] = previous_feedback
            ProxyActionClient._result_status[topic] = previous_status
            ProxyActionClient._has_active_goal[topic] = previous_has_active_goal
            ProxyActionClient._current_goal[topic] = previous_goal if previous_has_active_goal else None
            raise

    @classmethod
    def _done_callback(cls, future, topic, client_generation):
        if cls._is_shutting_down or not cls._matches_client_generation(topic, client_generation):
            return
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                result = goal_handle.get_result_async()
                if ProxyActionClient._result_status.get(topic) != GoalStatus.STATUS_CANCELING:
                    ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ACCEPTED
                result.add_done_callback(partial(ProxyActionClient._result_callback,
                                                 topic=topic,
                                                 client_generation=client_generation))
            else:
                ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ABORTED
                ProxyActionClient._has_active_goal[topic] = False
                ProxyActionClient._current_goal[topic] = None
                Logger.localinfo(f"Goal for '{topic}' ({goal_handle.goal_id.uuid}) was rejected!")
        except Exception as exc:  # pylint: disable=W0703
            ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ABORTED
            ProxyActionClient._has_active_goal[topic] = False
            ProxyActionClient._current_goal[topic] = None
            Logger.localwarn(f"Goal completion callback for '{topic}' failed: {type(exc).__name__} - {exc}")

    @classmethod
    def _result_callback(cls, future, topic, client_generation):
        if cls._is_shutting_down or not cls._matches_client_generation(topic, client_generation):
            return
        try:
            response = future.result()
            ProxyActionClient._result[topic] = response.result
            ProxyActionClient._result_status[topic] = response.status
        except Exception as exc:  # pylint: disable=W0703
            ProxyActionClient._result[topic] = None
            ProxyActionClient._result_status[topic] = GoalStatus.STATUS_ABORTED
            Logger.localwarn(f"Result callback for '{topic}' failed: {type(exc).__name__} - {exc}")
        ProxyActionClient._has_active_goal[topic] = False
        ProxyActionClient._current_goal[topic] = None

    @classmethod
    def _feedback_callback(cls, topic, feedback, client_generation):
        if cls._is_shutting_down or not cls._matches_client_generation(topic, client_generation):
            return
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
            return

        # Clear old data
        ProxyActionClient._feedback[topic] = None
        ProxyActionClient._has_active_goal[topic] = False
        ProxyActionClient._current_goal[topic] = None
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
        previous_status = ProxyActionClient._result_status.get(topic)
        previous_has_active_goal = ProxyActionClient._has_active_goal.get(topic, False)
        previous_goal = ProxyActionClient._current_goal.get(topic)
        try:
            current_goal_future = previous_goal
            if current_goal_future is None:
                raise ProxyAvailabilityError(f"No active goal future for '{topic}'.")

            ProxyActionClient._result_status[topic] = GoalStatus.STATUS_CANCELING
            client_generation = cls._get_client_generation(topic)

            if hasattr(current_goal_future, 'done') and not current_goal_future.done():
                current_goal_future.add_done_callback(partial(ProxyActionClient._cancel_when_goal_ready,
                                                              topic=topic,
                                                              client_generation=client_generation,
                                                              previous_status=previous_status,
                                                              previous_has_active_goal=previous_has_active_goal,
                                                              previous_goal=previous_goal))
                return

            ProxyActionClient._cancel_goal_from_future(current_goal_future, topic, client_generation,
                                                       previous_status=previous_status,
                                                       previous_has_active_goal=previous_has_active_goal,
                                                       previous_goal=previous_goal)
        except (ProxyAvailabilityError, ProxyTypeError) as exc:
            Logger.localinfo(f"Failed to send cancel request for '{topic}' : {exc}")
            ProxyActionClient._restore_after_cancel_failure(topic, previous_status,
                                                            previous_has_active_goal, previous_goal)
        except RuntimeError as exc:
            Logger.localinfo(f"  Error canceling '{topic}' : {exc}")
            ProxyActionClient._restore_after_cancel_failure(topic, previous_status,
                                                            previous_has_active_goal, previous_goal)

    @classmethod
    def _cancel_when_goal_ready(cls, future, topic, client_generation,
                                previous_status=None, previous_has_active_goal=False, previous_goal=None):
        """Issue the queued cancel request once the goal handle becomes available."""
        if cls._is_shutting_down or not cls._matches_client_generation(topic, client_generation):
            return
        try:
            ProxyActionClient._cancel_goal_from_future(future, topic, client_generation,
                                                       previous_status=previous_status,
                                                       previous_has_active_goal=previous_has_active_goal,
                                                       previous_goal=previous_goal)
        except (ProxyAvailabilityError, ProxyTypeError) as exc:
            Logger.localinfo(f"Failed queued cancel request for '{topic}' : {exc}")
            ProxyActionClient._restore_after_cancel_failure(topic, previous_status,
                                                            previous_has_active_goal, previous_goal)
        except RuntimeError as exc:
            Logger.localinfo(f"  Error canceling '{topic}' after goal acceptance: {exc}")
            ProxyActionClient._restore_after_cancel_failure(topic, previous_status,
                                                            previous_has_active_goal, previous_goal)

    @classmethod
    def _cancel_goal_from_future(cls, current_goal_future, topic, client_generation,
                                 previous_status=None, previous_has_active_goal=False, previous_goal=None):
        """Resolve the current goal future and send a cancel request."""
        current_goal_handle = ProxyActionClient._resolve_goal_handle_for_cancel(topic, current_goal_future)
        cancel_future = current_goal_handle.cancel_goal_async()
        if cancel_future is None:
            raise ProxyAvailabilityError(f"Cancel request for '{topic}' returned no future.")

        # add callback to acknowledge completion of cancel_goal
        cancel_future.add_done_callback(partial(ProxyActionClient._cancel_callback,
                                                topic=topic,
                                                client_generation=client_generation,
                                                previous_status=previous_status,
                                                previous_has_active_goal=previous_has_active_goal,
                                                previous_goal=previous_goal))

    @classmethod
    def _restore_after_cancel_failure(cls, topic, previous_status, previous_has_active_goal, previous_goal):
        """Restore pre-cancel tracking only if this cancel attempt still owns topic state."""
        if ProxyActionClient._current_goal.get(topic) is not previous_goal:
            return
        if ProxyActionClient._result_status.get(topic) != GoalStatus.STATUS_CANCELING:
            return
        ProxyActionClient._current_goal[topic] = previous_goal
        ProxyActionClient._has_active_goal[topic] = previous_has_active_goal
        ProxyActionClient._result_status[topic] = previous_status

    @classmethod
    def _resolve_goal_handle_for_cancel(cls, topic, current_goal_future=None):
        """Resolve and validate active goal handle before issuing cancel request."""
        if current_goal_future is None:
            current_goal_future = ProxyActionClient._current_goal.get(topic)
        if current_goal_future is None:
            raise ProxyAvailabilityError(f"No active goal future for '{topic}'.")
        current_goal_handle = current_goal_future.result()
        if current_goal_handle is None:
            raise ProxyAvailabilityError(f"No goal handle available for '{topic}'.")
        if not hasattr(current_goal_handle, 'cancel_goal_async'):
            raise ProxyTypeError(f"Goal handle for '{topic}' does not support cancel_goal_async().")
        return current_goal_handle

    @classmethod
    def _cancel_callback(cls, future, topic, client_generation,
                         previous_status=None, previous_has_active_goal=False, previous_goal=None):
        if cls._is_shutting_down or not cls._matches_client_generation(topic, client_generation):
            return
        try:
            result = future.result()
            Logger.localinfo(f"   cancel result for '{topic}' : result={result.return_code}")
            if result.return_code == CancelGoal.Response.ERROR_NONE:
                if ProxyActionClient._current_goal.get(topic) is previous_goal and \
                        ProxyActionClient._result_status.get(topic) == GoalStatus.STATUS_CANCELING:
                    ProxyActionClient._result_status[topic] = GoalStatus.STATUS_CANCELED
            else:
                Logger.localwarn(f"Cancel request for '{topic}' was not accepted (return_code={result.return_code}).")
                ProxyActionClient._restore_after_cancel_failure(topic, previous_status,
                                                                previous_has_active_goal, previous_goal)
        except Exception as exc:  # pylint: disable=W0703
            Logger.localwarn(f"Cancel callback for '{topic}' failed: {type(exc).__name__} - {exc}")
            ProxyActionClient._restore_after_cancel_failure(topic, previous_status,
                                                            previous_has_active_goal, previous_goal)

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

        warning_event = None
        tmr = None
        if wait_duration > 2.0:
            warning_event = Event()
            tmr = Timer(.5, ProxyActionClient._mark_wait_warning, [warning_event, topic])
            tmr.start()

        client = client_dict['client']
        try:
            available = client.wait_for_server(wait_duration)
        finally:
            if tmr is not None:
                tmr.cancel()

        warning_sent = wait_duration > 2.0 and warning_event.is_set()

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
    def _mark_wait_warning(cls, warning_event, topic):
        """Record that wait warning was emitted and log warning once."""
        warning_event.set()
        cls._print_wait_warning(topic)

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

                    if topic in ProxyActionClient._result_status:
                        ProxyActionClient._result_status.pop(topic)

                    if topic in ProxyActionClient._has_active_goal:
                        ProxyActionClient._has_active_goal.pop(topic)

                    if topic in ProxyActionClient._current_goal:
                        ProxyActionClient._current_goal.pop(topic)

        if client is not None:
            Logger.localdebug(f"Action client for '{topic}' has {count} references remaining.")
            ProxyActionClient._node.executor.create_task(ProxyActionClient.destroy_client, client, topic)
        else:
            Logger.localdebug(f"Action client for '{topic}' remains with {count} references!")

    @classmethod
    def destroy_client(cls, client, topic):
        """Handle client destruction from within the executor threads."""
        if client is None:
            return
        if cls._node is None:
            Logger.localwarn(f"Cannot destroy action client for '{topic}': proxy node is not initialized.")
            return
        try:
            if cls._node.destroy_client(client):
                Logger.localinfo(f"Destroyed the proxy action client for '{topic}'!")
            else:
                Logger.localwarn(f"Some issue destroying the proxy action client for '{topic}'!")
            del client
        except Exception as exc:  # pylint: disable=W0703
            Logger.error('Something went wrong destroying action client'
                         f" for '{topic}'!\n  {type(exc)} - {str(exc)}")
