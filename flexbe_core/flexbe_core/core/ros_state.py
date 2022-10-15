#!/usr/bin/env python
from rclpy.exceptions import ParameterNotDeclaredException
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from flexbe_core.core.state import State
from flexbe_core.logger import Logger
from flexbe_core.state_logger import StateLogger


class RosState(State):
    """
    A state to interface with ROS.
    """
    _node = None
    _breakpoints = []

    @staticmethod
    def initialize_ros(node):
        RosState._node = node
        try:
            RosState._breakpoints = node.get_parameter('breakpoints')
        except ParameterNotDeclaredException as e:
            node.declare_parameter('breakpoints', [])
            RosState._breakpoints = node.get_parameter('breakpoints')
        ProxyPublisher._initialize(RosState._node)
        ProxySubscriberCached._initialize(RosState._node)
        StateLogger.initialize_ros(RosState._node)
        Logger.initialize(RosState._node)

    def __init__(self, *args, **kwargs):
        super(RosState, self).__init__(*args, **kwargs)

        self._desired_period_ns = (1 / 10) * 1e9

        if "desired_rate" in kwargs:
            Logger.localinfo(f'RosState: Set desired state update rate to {kwargs["desired_rate"]} Hz.')
            self._desired_period_ns = (1 / kwargs["desired_rate"]) * 1e9

        self._is_controlled = False

        self._pub = ProxyPublisher()
        self._sub = ProxySubscriberCached()

        self._last_execution = None

    @property
    def sleep_duration(self):
        """
        Sleep duration in seconds
        """
        if self._last_execution is None:
            return -1  # No sleep if not executed since last entry

        elapsed = RosState._node.get_clock().now() - self._last_execution

        # Take how long the timer should sleep for and subtract elapsed time
        return (self._desired_period_ns - elapsed.nanoseconds) * 1e-9

    def set_rate(self, desired_rate):
        """
        Set the execution rate of this state,
        i.e., the rate with which the execute method is being called.

        Note: The rate is best-effort, real-time support is not yet available.

        @type desired_rate: float
        @param desired_rate: The desired rate in Hz.
        """
        self._desired_period_ns = (1 / desired_rate) * 1e9

    def _enable_ros_control(self):
        self._is_controlled = True

    def _disable_ros_control(self):
        self._is_controlled = False

    @property
    def is_breakpoint(self):
        return self.path in RosState._breakpoints.get_parameter_value().string_array_value
