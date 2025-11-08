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


"""Realize behavior-specific logging."""

from flexbe_core.core.topics import Topics

from flexbe_msgs.msg import BehaviorLog

from rclpy.duration import Duration
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node


class Logger:
    """Realize behavior-specific logging."""

    REPORT_INFO = BehaviorLog.INFO
    REPORT_WARN = BehaviorLog.WARN
    REPORT_HINT = BehaviorLog.HINT
    REPORT_ERROR = BehaviorLog.ERROR
    REPORT_DEBUG = BehaviorLog.DEBUG

    # max number of items in last logged dict (used for log_throttle)
    MAX_LAST_LOGGED_SIZE = 1024
    LAST_LOGGED_CLEARING_RATIO = 0.2

    _pub = None
    _node = None

    @staticmethod
    def initialize(node: Node):
        """Initialize the logger instance."""
        Logger._node = node
        Logger._pub = node.create_publisher(BehaviorLog, Topics._BEHAVIOR_LOGGING_TOPIC, 100)
        Logger._last_logged = {}

        # Optional parameters that can be defined
        try:
            size_param = node.get_parameter('max_throttle_logging_size')
            if size_param.type_ == size_param.Type.INTEGER:
                Logger.MAX_LAST_LOGGED_SIZE = size_param.value
        except ParameterNotDeclaredException:
            pass

        try:
            clear_param = node.get_parameter('throttle_logging_clear_ratio')
            if clear_param.type_ in (clear_param.Type.INTEGER, clear_param.Type.DOUBLE):
                Logger.LAST_LOGGED_CLEARING_RATIO = clear_param.value
        except ParameterNotDeclaredException:
            pass

        Logger._node.get_logger().debug(f'Enable throttle logging option with '
                                        f'max size={Logger.MAX_LAST_LOGGED_SIZE} '
                                        f'clear ratio={Logger.LAST_LOGGED_CLEARING_RATIO}')

    @staticmethod
    def log(text: str, severity: int):
        """Log message."""
        if Logger._node is None:
            raise RuntimeError('Unable to log, run "Logger.initialize" first to define the target ROS node.')
        # send message with logged text
        msg = BehaviorLog()
        msg.text = str(text)
        msg.status_code = severity
        Logger._pub.publish(msg)
        # also log locally
        Logger.local(text, severity)

    @staticmethod
    def log_throttle(period: float, text: str, severity: int):
        """Log unique messages once and don't repeat messages."""
        # create unique identifier for each logging message
        log_id = f'{severity}_{text}'
        time_now = Logger._node.get_clock().now()
        # only log when it's the first time or period time has passed for the logging message
        if log_id not in Logger._last_logged.keys() or \
           time_now - Logger._last_logged[log_id] > Duration(seconds=period):
            Logger.log(text, severity)
            Logger._last_logged.update({log_id: time_now})

        if len(Logger._last_logged) > Logger.MAX_LAST_LOGGED_SIZE:
            # iterate through last logged items, sorted by the timestamp (oldest last)
            clear_size = Logger.MAX_LAST_LOGGED_SIZE * (1 - Logger.LAST_LOGGED_CLEARING_RATIO)
            for i, log in enumerate(sorted(Logger._last_logged.items(), key=lambda item: item[1], reverse=True)):
                # remove defined percentage of oldest items
                if i > clear_size:
                    Logger._last_logged.pop(log[0])

    @staticmethod
    def local(text: str, severity: int):
        """Local logging to terminal."""
        if Logger._node is None:
            raise RuntimeError('Unable to log, run "Logger.initialize" first to define the target ROS node.')
        if severity == Logger.REPORT_INFO:
            Logger._node.get_logger().info(text)
        elif severity == Logger.REPORT_WARN:
            Logger._node.get_logger().warning(f'\033[93m{text}\033[0m')
        elif severity == Logger.REPORT_HINT:
            Logger._node.get_logger().info(f'\033[94mBehavior Hint: {text}\033[0m')
        elif severity == Logger.REPORT_ERROR:
            Logger._node.get_logger().error(f'\033[91m{text}\033[0m')
        elif severity == Logger.REPORT_DEBUG:
            Logger._node.get_logger().debug(f'\033[92m{text}\033[0m')
        else:
            Logger._node.get_logger().debug(f'\033[95m{text}\033[91m(unknown log level {str(severity)})\033[0m')

    # NOTE: Below text strings can only have single % symbols if they are being treated
    # as format strings with appropriate arguments (otherwise replace with %% for simple string without args)
    @staticmethod
    def logdebug(text: str, *args):
        """Log debug."""
        Logger.log(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def loginfo(text: str, *args):
        """Log info."""
        Logger.log(text % args, Logger.REPORT_INFO)

    @staticmethod
    def logwarn(text: str, *args):
        """Log warn."""
        Logger.log(text % args, Logger.REPORT_WARN)

    @staticmethod
    def loghint(text: str, *args):
        """Log hint."""
        Logger.log(text % args, Logger.REPORT_HINT)

    @staticmethod
    def logerr(text: str, *args):
        """Log error."""
        Logger.log(text % args, Logger.REPORT_ERROR)

    @staticmethod
    def logdebug_throttle(period: float, text: str, *args):
        """Log debug throttle."""
        Logger.log_throttle(period, text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def loginfo_throttle(period: float, text: str, *args):
        """Log info throttle."""
        Logger.log_throttle(period, text % args, Logger.REPORT_INFO)

    @staticmethod
    def logwarn_throttle(period: float, text: str, *args):
        """Log warn throttle."""
        Logger.log_throttle(period, text % args, Logger.REPORT_WARN)

    @staticmethod
    def loghint_throttle(period: float, text: str, *args):
        """Log hint throttle."""
        Logger.log_throttle(period, text % args, Logger.REPORT_HINT)

    @staticmethod
    def logerr_throttle(period: float, text: str, *args):
        """Log error throttle."""
        Logger.log_throttle(period, text % args, Logger.REPORT_ERROR)

    @staticmethod
    def localdebug(text: str, *args):
        """Local debug."""
        Logger.local(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def localinfo(text: str, *args):
        """Local info."""
        Logger.local(text % args, Logger.REPORT_INFO)

    @staticmethod
    def localwarn(text: str, *args):
        """Local warn."""
        Logger.local(text % args, Logger.REPORT_WARN)

    @staticmethod
    def localhint(text: str, *args):
        """Local hint."""
        Logger.local(text % args, Logger.REPORT_HINT)

    @staticmethod
    def localerr(text: str, *args):
        """Local error."""
        Logger.local(text % args, Logger.REPORT_ERROR)

    @staticmethod
    def debug(text: str, *args):
        """Log debug."""
        Logger.logdebug(text, *args)

    @staticmethod
    def info(text: str, *args):
        """Log info."""
        Logger.loginfo(text, *args)

    @staticmethod
    def warning(text: str, *args):
        """Log warning."""
        Logger.logwarn(text, *args)

    @staticmethod
    def hint(text: str, *args):
        """Log hint."""
        Logger.loghint(text, *args)

    @staticmethod
    def error(text: str, *args):
        """Log error."""
        Logger.logerr(text, *args)
