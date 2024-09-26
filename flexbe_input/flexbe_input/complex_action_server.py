#! /usr/bin/env python

# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
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
#    * Neither the name of the Willow Garage, Inc. nor the names of its
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


# Author: Brian Wright.
# Based on C++ simple_action_server.h by Eitan Marder-Eppstein


import queue
import threading
import traceback

from flexbe_core import Logger

import rclpy
from rclpy.action import ActionServer
from rclpy.duration import Duration


def nop_cb(goal_handle):
    """Execute no operation (pass through) callback."""
    pass


# @class ComplexActionServer
# @brief The ComplexActionServer
# Operate with concurrent goals in a multi-threaded fashion


class ComplexActionServer:
    """
    Constructor for a ComplexActionServer.

    @param name A name for the action server
    @param execute_cb Optional callback that gets called in a separate thread whenever
                      a new goal is received, allowing users to have blocking callbacks.
                      Adding an execute callback also deactivates the goalCallback.
    @param  auto_start A boolean value that tells the ActionServer whether or not
                       to start publishing as soon as it comes up.
                       THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and
                       start() should be called after construction of the server.
    """

    def __init__(self, node, name, ActionSpec, execute_cb=None, auto_start=False):
        """Initialize instance of the ComplexActionServer."""
        self.node = node
        self.goals_received_ = 0
        self.goal_queue_ = queue.Queue()

        self.new_goal = False

        self.execute_callback = execute_cb
        self.goal_callback = None

        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()

        # since the internal_goal/preempt_callbacks are invoked from the
        # ActionServer while holding the self.action_server.lock
        # self.lock must always be locked after the action server lock
        # to avoid an inconsistent lock acquisition order
        self.lock = threading.RLock()

        self.execute_condition = threading.Condition(self.lock)

        self.current_goal = None
        self.next_goal = None

        if self.execute_callback:
            self.execute_thread = threading.Thread(None, self.executeLoop)
            self.execute_thread.start()
        else:
            self.execute_thread = None

        # create the action server
        self.action_server = ActionServer(node,
                                          action_type=ActionSpec,
                                          action_name=name,
                                          execute_callback=nop_cb,
                                          goal_callback=self.internal_goal_callback,
                                          cancel_callback=self.internal_preempt_callback)

    def __del__(self):
        if hasattr(self, 'execute_callback') and self.execute_callback:
            with self.terminate_mutex:
                self.need_to_terminate = True

            assert(self.execute_thread)
            self.execute_thread.join()

    # @brief Accepts a new goal when one is available The status of this
    # goal is set to active upon acceptance,
    def accept_new_goal(self):
        """Accept a new goal."""
        # with self.action_server.lock, self.lock:

        Logger.logdebug('Accepting a new goal')

        self.goals_received_ -= 1

        # get from queue
        current_goal = self.goal_queue_.get()

        # set the status of the current goal to be active
        # current_goal.set_accepted('This goal has been accepted by the simple action server');
        current_goal.succeed()

        return current_goal

    # @brief Allows  polling implementations to query about the availability of a new goal
    # @return True if a new goal is available, false otherwise
    def is_new_goal_available(self):
        """Check if new goal is available."""
        return self.goals_received_ > 0

    # @brief Allows  polling implementations to query about the status of the current goal
    # @return True if a goal is active, false otherwise
    def is_active(self):
        """Check if current goal is active."""
        if self.current_goal and not self.current_goal.get_goal():
            return False

        return self.current_goal.is_active

    # @brief Sets the status of the active goal to succeeded
    # @param  result An optional result to send back to any clients of the goal
    def set_succeeded(self, result=None, text='', goal_handle=None):
        """Set status to success."""
        goal_handle.succeed()
        if not result:
            result = self.get_default_result()

        return result

    # @brief Sets the status of the active goal to aborted
    # @param  result An optional result to send back to any clients of the goal
    def set_aborted(self, result=None, text='', goal_handle=None):
        """Set goal aborted."""
        goal_handle.abort()
        if not result:
            result = self.get_default_result()

        return result

    # @brief Publishes feedback for a given goal
    # @param  feedback Shared pointer to the feedback to publish
    def publish_feedback(self, feedback):
        """Publish feedback."""
        self.current_goal.publish_feedback(feedback)

    def get_default_result(self):
        """Get result."""
        return self.action_server.action_type

    # @brief Allows users to register a callback to be invoked when a new goal is available
    # @param cb The callback to be invoked
    def register_goal_callback(self, cb):
        """Register callback."""
        if self.execute_callback:
            Logger.logwarn('Cannot call ComplexActionServer.register_goal_callback() '
                           'because an executeCallback exists. Not going to register it.')
        else:
            self.goal_callback = cb

    # @brief Callback for when the ActionServer receives a new goal and passes it on
    def internal_goal_callback(self, goal):
        """Call   when the ActionServer receives a new goal and passes it on."""
        self.execute_condition.acquire()

        try:
            Logger.localinfo(f'A new goal {goal.goal_id} has been received by the single goal action server')

            self.next_goal = goal
            self.new_goal = True
            self.goals_received_ += 1

            # add goal to queue
            self.goal_queue_.put(goal)

            # Trigger runLoop to call execute()
            self.execute_condition.notify()
            self.execute_condition.release()

        except Exception as e:
            Logger.logerr('ComplexActionServer.internal_goal_callback - exception %s', str(e))
            self.execute_condition.release()

    # @brief Callback for when the ActionServer receives a new preempt and passes it on
    def internal_preempt_callback(self, preempt):
        """Call when the ActionServer receives a new preempt and passes it on."""
        return

    # @brief Called from a separate thread to call blocking execute calls
    def executeLoop(self):
        """Execute the server loop."""
        loop_duration = Duration(seconds=0.1)

        while (rclpy.ok()):
            Logger.logdebug('ComplexActionServer: execute')

            with self.terminate_mutex:
                if (self.need_to_terminate):
                    break

            if (self.is_new_goal_available()):
                goal_handle = self.accept_new_goal()
                if not self.execute_callback:
                    Logger.logerr('execute_callback_ must exist. This is a bug in ComplexActionServer')
                    return

                try:

                    print('run new executecb')
                    thread = threading.Thread(target=self.run_goal, args=(goal_handle.get_goal(), goal_handle))
                    thread.start()

                except Exception as ex:
                    Logger.logerr(f'Exception in your execute callback: {str(ex)}\n',
                                  f"{traceback.format_exc().replace('%', '%%')}")
                    self.set_aborted(None, f'Exception in execute callback: {str(ex)}')

            with self.execute_condition:
                self.execute_condition.wait(loop_duration.nanoseconds * 1.e-9)

    def run_goal(self, goal, goal_handle):
        """Run goal callback."""
        self.execute_callback(goal, goal_handle)
