#!/usr/bin/env python3

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


"""A state machine to interface with ROS."""
from flexbe_core.core.state_machine import StateMachine
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from rclpy.duration import Duration


class RosStateMachine(StateMachine):
    """A state machine to interface with ROS."""

    _node = None

    @staticmethod
    def initialize_ros(node):
        """Initialize ROS node information."""
        RosStateMachine._node = node
        ProxyPublisher.initialize(node)
        ProxySubscriberCached.initialize(node)

    def __init__(self, *args, **kwargs):
        """Initialize instance of ROSStateMachine."""
        super().__init__(*args, **kwargs)
        self._is_controlled = False

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

    def _notify_start(self):
        """Call when state machine starts up."""

    def _notify_stop(self):
        """Call when state machine shuts down."""

    def wait(self, seconds=None, context=None):
        """
        Wait for designated ROS clock time to keep APPROXIMATELY on scheduled tic rate.

        @param seconds - floating point seconds to sleep
        @param context - rclpy Context, normally None for default context, but specific context used in testing
        """
        if seconds is not None and seconds > 0:
            self._node.get_clock().sleep_for(Duration(seconds=seconds), context=context)

    def _enable_ros_control(self):
        if not self._is_controlled:
            self._is_controlled = True
            for state in self._states:
                state._enable_ros_control()

    def _disable_ros_control(self):
        if self._is_controlled:
            self._is_controlled = False
            for state in self._states:
                state._disable_ros_control()
