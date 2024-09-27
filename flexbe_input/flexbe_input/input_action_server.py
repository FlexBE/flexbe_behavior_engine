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

import ast
import pickle
import time

from PySide6.QtCore import QCoreApplication, QThread, Qt, Signal, Slot
from PySide6.QtWidgets import QApplication

from flexbe_core import Logger

from flexbe_input.input_gui import InputGUI

from flexbe_msgs.action import BehaviorInput

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class InputActionWorker(QThread):
    """Worker thread for InputAction server."""

    _show_dialog_signal = Signal(str, object)
    _hide_dialog_signal = Signal()

    def __init__(self, node):
        super().__init__()
        self._node = node

    def run(self):
        """Run loop for worker thread."""
        try:
            # Use a MultiThreadedExecutor to enable processing goals concurrently
            executor = MultiThreadedExecutor()
            print('Begin spinning ROS loop for InputActionServer ...', flush=True)
            rclpy.spin(self._node, executor=executor)
        except (KeyboardInterrupt, ExternalShutdownException):
            print('Caught KeyboardInterrupt in InputActionWorker thread - shutdown ...')
        except Exception as exc:
            print(f'Unknown exception in InputActionWorker:\n    {exc}')

        print('\nShutting down the input action server ...', flush=True)
        QCoreApplication.quit()  # Trigger shutdown of GUI application


class InputActionServer(Node):
    """
    Implements the server callback for when a goal is requested by a FlexBE input state.

    Input can only handle Python primitives and lists.
    If a state requires an input key of a specific type, check for that type from the state.
    """

    def __init__(self):
        """Initialize the InputActionServer instance."""
        super().__init__('input_action_server')
        self._action_topic = 'flexbe/behavior_input'
        self._server = ActionServer(
            self,
            BehaviorInput,
            self._action_topic,
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )

        self._input_dialog = InputGUI('default')

        self._input = None
        self._canceled = False
        self._worker = None

        Logger.initialize(self)

    def get_input_type(self, request_type):
        """
        Get input prompt for data.

        @param request_type

        @return prompt, instance type, number of elements
        """
        # These are the only types handled by this simple UI
        types = {BehaviorInput.Goal.REQUEST_INT: ('int', int, 1),
                 BehaviorInput.Goal.REQUEST_FLOAT: ('float', (float, int), 1),  # int acceptable for desired float
                 BehaviorInput.Goal.REQUEST_2D: ('list of 2 numbers', (list, tuple), 2),  # allow either list or tuple
                 BehaviorInput.Goal.REQUEST_3D: ('list of 3 numbers', (list, tuple), 3),  # e.g., '[1, 2]', '(1, 2)', or '1, 2'
                 BehaviorInput.Goal.REQUEST_4D: ('list of 4 numbers', (list, tuple), 4),
                 BehaviorInput.Goal.REQUEST_STRING: ('string', str, 1),
                 BehaviorInput.Goal.REQUEST_SELECTION: ('selected item', str, 1)
                 }

        if request_type in types:
            return types[request_type]

        return None

    def execute_callback(self, goal_handle):
        """On receipt of goal, open GUI and request input from user."""
        # Reset cancellation flag
        self._canceled = False

        result = BehaviorInput.Result()
        Logger.localinfo('Requesting: %s', goal_handle.request.msg)
        # Logger.localinfo(f'   goal id: {goal_handle.goal_id.uuid}')
        try:
            type_text, type_class, expected_elements = self.get_input_type(goal_handle.request.request_type)
            prompt_text = f'{goal_handle.request.msg}\n{type_text}'
        except Exception as exc:  # pylint: disable=W0703
            result.data = f'Input action server UI does not handle requests for type {goal_handle.request.request_type}'
            result.result_code = BehaviorInput.Result.RESULT_ABORTED
            Logger.localwarn(f'{result.data}\n    Exception: {exc}')
            goal_handle.abort()
            return result

        # Get data from user
        try:
            # Request input from GUI running in a separate thread
            if goal_handle.request.request_type == BehaviorInput.Goal.REQUEST_SELECTION:
                self._worker._show_dialog_signal.emit(prompt_text, goal_handle.request.items)
            else:
                self._worker._show_dialog_signal.emit(prompt_text, None)

            while self._input_dialog.is_none() and not self._canceled:
                time.sleep(0.02)  # Add a short sleep to avoid busy-waiting

            # Emit signal to hide the dialog and get input
            self._worker._hide_dialog_signal.emit()

            if self._canceled:
                Logger.localwarn('Request was canceled!')
                goal_handle.canceled()
                result.data = 'Input request was canceled!'
                result.result_code = BehaviorInput.Result.RESULT_ABORTED
                return result

            if self._input is None or self._input == '':
                Logger.logwarn(f"No data entered while input window was visible! '{self._input}'")
                result.result_code = BehaviorInput.Result.RESULT_ABORTED
                result.data = 'No data entered while input window was visible!'
                goal_handle.abort()
                return result
            else:
                if type_class is str:
                    print(f"Process data as string '{self._input}' with request {type_class}", flush=True)
                    result.data = self._input
                    data_len = 1
                else:
                    print(f"Process data '{self._input}' as {type_class}", flush=True)
                    input_data = ast.literal_eval(self._input)  # convert string to Python data
                    print(f"  input data[{type(input_data)}] = '{input_data}'", flush=True)
                    data_len = 1 if isinstance(input_data, (int, float)) else len(input_data)

                    if not isinstance(input_data, type_class):
                        result.data = f"Invalid input type '{type(result.data)}' not '{type_class}' - expected '{type_text}'"
                        result.result_code = BehaviorInput.Result.RESULT_FAILED
                        Logger.localwarn(result.data)
                        goal_handle.abort()
                        return result
                    # Convert binary to string for transport
                    result.data = str(pickle.dumps(input_data))

                if data_len != expected_elements:
                    result.data = (f'Invalid number of elements {data_len} not {expected_elements} '
                                   f"of {type_class} - expected '{type_text}'")
                    result.result_code = BehaviorInput.Result.RESULT_FAILED
                    Logger.localwarn(result.data)
                    goal_handle.abort()
                else:
                    result.result_code = BehaviorInput.Result.RESULT_OK
                    goal_handle.succeed()
        except Exception as exc:  # pylint: disable=W0703
            Logger.logwarn(f'Error processing input request to set data:\n {exc}')
            result.result_code = BehaviorInput.Result.RESULT_FAILED
            result.data = str(exc)
            goal_handle.abort()

        self._canceled = False  # Reset cancellation flag
        return result

    def cancel_callback(self, goal_handle):
        """Cancel the active goal."""
        Logger.localwarn(f"Canceling goal for '{self._action_topic}' ...")
        self._canceled = True
        return rclpy.action.CancelResponse.ACCEPT

    @Slot()
    def on_get_input(self):
        """Get the input from edit box."""
        self._input = self._input_dialog.get_input()


def main(args=[]):
    """Run action server and GUI on request."""
    app = QApplication(args)

    rclpy.init()
    action_server = InputActionServer()

    # Connect signals and slots
    worker = InputActionWorker(action_server)
    action_server._worker = worker  # Allow server to access signals
    worker._show_dialog_signal.connect(action_server._input_dialog.show, type=Qt.BlockingQueuedConnection)
    worker._hide_dialog_signal.connect(action_server._input_dialog.hide, type=Qt.BlockingQueuedConnection)
    worker._hide_dialog_signal.connect(action_server.on_get_input, type=Qt.BlockingQueuedConnection)
    worker.start()

    try:
        print('Begin GUI execution for InputActionServer ...', flush=True)
        app.exec_()
    except Exception as e:
        print(f'Exception in app.exec_: {e}', flush=True)

    print('requesting rclpy shutdown ...', flush=True)
    rclpy.shutdown()

    print('Ensure shutdown of ROS worker thread ...', flush=True)
    worker.quit()

    print('wait on ROS thread to close ...', flush=True)
    worker.wait()
    print('done!', flush=True)


if __name__ == '__main__':
    main()
