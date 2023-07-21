# Copyright 2023 Christopher Newport University
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

import ast
import pickle
import sys
import time

from PyQt5 import QtWidgets

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from flexbe_core import Logger
from flexbe_input.input_gui import InputGUI
from flexbe_msgs.action import BehaviorInput


class InputActionServer(Node):
    """
    Implements the server callback for when a goal is requested by a FlexBE input state.

    Input can only handle Python primitives and lists.
    If a state requires an input key of a specific type, check for that type from the state.
    """

    def __init__(self):
        super().__init__('input_action_server')
        self._server = ActionServer(
            self,
            BehaviorInput,
            'flexbe/behavior_input',
            self.execute_callback
        )
        self._app = None
        self._input = None
        Logger.initialize(self)

    def get_input_type(self, request_type):
        """
        Get input prompt for data.

        @param request_type

        @return prompt, instance type, number of elements
        """
        # Thse are the only types handled by this simple UI
        types = {BehaviorInput.Goal.REQUEST_INT: ("int", int, 1),
                 BehaviorInput.Goal.REQUEST_FLOAT: ("float", (float, int), 1),  # int acceptable for desired float
                 BehaviorInput.Goal.REQUEST_2D: ("list of 2 numbers", (list, tuple), 2),  # allow either list or tuple
                 BehaviorInput.Goal.REQUEST_3D: ("list of 3 numbers", (list, tuple), 3),  # e.g., "[1, 2]", "(1, 2)", or "1, 2"
                 BehaviorInput.Goal.REQUEST_4D: ("list of 4 numbers", (list, tuple), 4),
                 }

        if request_type in types:
            return types[request_type]

        return None

    def execute_callback(self, goal_handle):

        result = BehaviorInput.Result()
        Logger.localinfo("Requesting: %s", goal_handle.request.msg)
        try:
            type_text, type_class, expected_elements = self.get_input_type(goal_handle.request.request_type)
            prompt_text = f"{goal_handle.request.msg}\n{type_text}"
        except Exception as exc:  # pylint: disable=W0703
            result.data = f"Input action server UI does not handle requests for type {goal_handle.request.request_type}"
            result.result_code = BehaviorInput.Result.RESULT_ABORTED
            Logger.localwarn(f"{result.data}\n    Exception: {exc}")
            goal_handle.abort()
            return result

        # Get data from user
        app = QtWidgets.QApplication(sys.argv)
        mainWin = InputGUI(prompt_text)
        mainWin.show()
        while mainWin.is_none() and mainWin.isVisible():
            QtWidgets.qApp.processEvents()
            time.sleep(.05)
        self._input = mainWin.get_input()
        mainWin.close()
        app.quit()

        if self._input is None:
            Logger.logwarn("No data entered while input window was visible!")
            result.result_code = BehaviorInput.Result.RESULT_ABORTED
            result.data = "No data entered while input window was visible!"
            goal_handle.abort()
        else:
            try:
                input_data = ast.literal_eval(self._input)  # convert string to Python data
                if not isinstance(input_data, type_class):
                    result.data = f"Invalid input type '{type(input_data)}' not '{type_class}' - expected '{type_text}'"
                    result.result_code = BehaviorInput.Result.RESULT_FAILED
                    Logger.localwarn(result.data)
                    goal_handle.abort()
                    return result

                data_len = 1 if isinstance(input_data, (int, float)) else len(input_data)
                if data_len != expected_elements:
                    result.data = (f"Invalid number of elements {data_len} not {expected_elements} "
                                   f"of {type_class} - expected '{type_text}'")
                    result.result_code = BehaviorInput.Result.RESULT_FAILED
                    Logger.localwarn(result.data)
                    goal_handle.abort()
                    return result

                result.data = str(pickle.dumps(input_data))
                Logger.localinfo("    Data returned %s", self._input)
                result.result_code = BehaviorInput.Result.RESULT_OK
                goal_handle.succeed()
            except Exception as exc:  # pylint: disable=W0703
                Logger.logwarn("Failure to set data: %s", str(exc))
                result.result_code = BehaviorInput.Result.RESULT_FAILED
                result.data = str(exc)
                goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = InputActionServer()
    try:
        Logger.localinfo("Waiting for requests from FlexBE input state ...")
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    print("\nShutting down the input action server!", flush=True)


if __name__ == '__main__':
    main()
