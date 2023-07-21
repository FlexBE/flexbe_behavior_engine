#!/usr/bin/env python

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

import ast
import pickle

from flexbe_core import EventState, Logger
from flexbe_msgs.action import BehaviorInput
from flexbe_core.proxy import ProxyActionClient


class InputState(EventState):
    """
    Implements a state where the state machine needs an input from the operator.

    Requests of different types, such as requesting a waypoint, a template, or a pose, can be specified.

    -- request  uint8       One of the custom-defined values to specify the type of request.
    -- message  string      Message displayed to the operator to let him know what to do.
    -- timeout  float       Timeout in seconds to wait for server to be available.

    #> data     object      The data provided by the operator. The exact type depends on the request.

    <= received             Returned as soon as valid data is available.
    <= aborted              The operator declined to provide the requested data.
    <= no_connection        No request could be sent to the operator.
    <= data_error           Data has been received, but could not be deserialized successfully.

    Note: This state uses the Pickle module, and is subject to this warning from the Pickle manual:
        Warning: The pickle module is not secure against erroneous or maliciously constructed data. 
        Never unpickle data received from an untrusted or unauthenticated source.

    If using this state to accept user input, it is up to the user to protect their network from untrusted data!

    """

    def __init__(self, request, message, timeout=1.0):
        """Construct instance."""
        super(InputState, self).__init__(outcomes=['received', 'aborted', 'no_connection', 'data_error'],
                                         output_keys=['data'])
        self._action_topic = 'flexbe/behavior_input'
        ProxyActionClient.initialize(InputState._node)
        self._client = ProxyActionClient({self._action_topic: BehaviorInput}, wait_duration=0.0)
        self._request = request
        self._message = message
        self._timeout = timeout
        self._connected = True
        self._received = False

    def execute(self, userdata):

        if not self._connected:
            return 'no_connection'
        if self._received:
            return 'received'

        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)
            if result.result_code != BehaviorInput.Result.RESULT_OK:
                userdata.data = None
                return 'aborted'
            else:
                # Attempt to load data and convert it to the proper format.
                try:
                    # Convert string to byte array and load using pickle
                    input_data = ast.literal_eval(result.data)

                    # Note: This state uses the Pickle module, and is subject to this warning from the Pickle manual:
                    #     Warning: The pickle module is not secure against erroneous or maliciously constructed data.
                    #     Never unpickle data received from an untrusted or unauthenticated source.
                    response_data = pickle.loads(input_data)

                    Logger.localinfo(f" InputState returned {type(response_data)} : {response_data}")
                    userdata.data = response_data
                except Exception as exc:  # pylint: disable=W0703
                    Logger.logwarn(f"Was unable to load provided data:\n    '{result.data}'\n    {str(exc)}")
                    userdata.data = None
                    return 'data_error'

                self._received = True
                return 'received'

        return None

    def on_enter(self, userdata):
        self._received = False

        # Retrive the goal for the BehaviorInput Action.
        action_goal = BehaviorInput.Goal()
        # Retrive the request type and message from goal.
        action_goal.request_type = self._request
        action_goal.msg = self._message
        Logger.loghint(f"Onboard requests '{self._message}'")

        # Attempt to send the goal.
        try:
            self._client.send_goal(self._action_topic, action_goal, wait_duration=self._timeout)
        except Exception as e:
            Logger.logwarn('Was unable to send data request:\n%s' % str(e))
            self._connected = False
