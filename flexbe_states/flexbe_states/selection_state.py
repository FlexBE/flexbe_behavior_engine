#!/usr/bin/env python3

# Copyright 2024 Christopher Newport University
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
#    * Neither the name of the Christopher Newport University nor the names of its
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

"""SelectionState."""
from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from flexbe_msgs.action import BehaviorInput


class SelectionState(EventState):
    """
    Implements a state where the state machine needs an input from the operator as choice from list of strings.

    Requests of different types, such as requesting a waypoint, a template, or a pose, can be specified.

    -- message  string      Message displayed to the operators to let them know what to do.
    -- timeout  float       Timeout in seconds to wait for server to be available.

    ># items    object      List or tuple of items to select from
    #> data     object      The data selected by the operator. The exact type depends on the request.

    <= received             Returned as soon as valid data is available.
    <= aborted              The operator declined to provide the requested data.
    <= no_connection        No request could be sent to the operator.
    <= data_error           Data has been received, but could not be deserialized successfully.

    Note: This state uses the Pickle module, and is subject to this warning from the Pickle manual:
        Warning: The pickle module is not secure against erroneous or maliciously constructed data.
        Never unpickle data received from an untrusted or unauthenticated source.

    If using this state to accept user input, it is up to the user to protect their network from untrusted data!

    """

    def __init__(self, message, timeout=1.0, action_topic='flexbe/behavior_input'):
        """Construct instance."""
        super(SelectionState, self).__init__(outcomes=['received', 'aborted', 'no_connection', 'data_error'],
                                             input_keys=['items'],
                                             output_keys=['data'])
        ProxyActionClient.initialize(SelectionState._node)
        self._action_topic = action_topic
        self._client = None
        self._message = message
        self._timeout = timeout
        self._return = None

    def on_start(self):
        """Set up proxy action client for behavior input server on behavior start."""
        self._client = ProxyActionClient({self._action_topic: BehaviorInput}, wait_duration=0.0)

    def on_stop(self):
        """Stop client when behavior stops."""
        ProxyActionClient.remove_client(self._action_topic)
        self._client = None

    def execute(self, userdata):
        """Execute state waiting for action result."""
        if self._return:
            # Return prior value if blocked
            return self._return

        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)
            if result.result_code != BehaviorInput.Result.RESULT_OK:
                userdata.data = None
                self._return = 'aborted'
            else:
                # Attempt to load data and convert it to the proper format.
                try:
                    # Selection state returns string
                    response_data = result.data

                    Logger.loginfo(f" SelectionState returned '{response_data}'")
                    userdata.data = response_data
                    self._return = 'received'
                except Exception as exc:  # pylint: disable=W0703
                    Logger.logwarn('Was unable to load provided data for '
                                   f"'{self._action_topic}':\n    '{result.data}'\n  "
                                   f' {str(exc)}')
                    userdata.data = None
                    self._return = 'data_error'
        elif self._client.get_status(self._action_topic) == GoalStatus.STATUS_CANCELED:
            Logger.localinfo(f" InputState {self._action_topic}' goal was canceled! ")
            self._return = 'aborted'
        elif self._client.get_status(self._action_topic) == GoalStatus.STATUS_ABORTED:
            Logger.localinfo(f" InputState {self._action_topic}' goal was aborted! ")
            self._return = 'aborted'

        return self._return

    def on_enter(self, userdata):
        """Send goal to action server on entering state."""
        self._client.remove_result(self._action_topic)
        self._return = None

        if 'items' not in userdata:
            self._return = 'aborted'
            msg = f"SelectionState '{self}' requires userdata.items key!"
            Logger.localwarn(msg)
            return

        # Retrieve the goal for the BehaviorInput Action.
        action_goal = BehaviorInput.Goal(request_type=BehaviorInput.Goal.REQUEST_SELECTION,
                                         items=userdata.items, msg=self._message)
        Logger.loghint(f"Onboard requests '{self._message}' : {userdata.items}")

        # Attempt to send the goal.
        try:
            self._client.send_goal(self._action_topic, action_goal, wait_duration=self._timeout)
            # Logger.localinfo(f"Sent goal for '{self._action_topic}'.")
        except Exception as exc:
            Logger.logwarn(f"Was unable to send data request for '{self._action_topic}':\n    {exc}")
            self._return = 'no_connection'

    def on_exit(self, userdata):
        """Call when state exits."""
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.
        Logger.localinfo(f"on exit for '{self._action_topic}'.")

        if self._client.is_active(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo(f"Requested to cancel active action goal for '{self._action_topic}'.")
