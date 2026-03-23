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
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.logging import LoggingSeverity
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
    _ros_logger = None
    _last_logged = {}
    _local_info_enabled = True
    _local_warn_enabled = True
    _local_hint_enabled = True
    _local_error_enabled = True
    _local_debug_enabled = False

    @staticmethod
    def _refresh_parameters(node: Node):
        """Refresh optional throttle logging parameters from the node."""
        Logger._last_logged = {}
        changed = False

        # Optional parameters that can be defined
        try:
            size_param = node.get_parameter('max_throttle_logging_size')
            if size_param.type_ == size_param.Type.INTEGER and Logger.MAX_LAST_LOGGED_SIZE != size_param.value:
                Logger.MAX_LAST_LOGGED_SIZE = size_param.value
                changed = True
        except ParameterNotDeclaredException:
            pass

        try:
            clear_param = node.get_parameter('throttle_logging_clear_ratio')
            if (
                clear_param.type_ in (clear_param.Type.INTEGER, clear_param.Type.DOUBLE)
                and Logger.LAST_LOGGED_CLEARING_RATIO != clear_param.value
            ):
                Logger.LAST_LOGGED_CLEARING_RATIO = clear_param.value
                changed = True
        except ParameterNotDeclaredException:
            pass

        if changed:
            Logger._ros_logger.debug(f'Enable throttle logging option with '
                                     f'max size={Logger.MAX_LAST_LOGGED_SIZE} '
                                     f'clear ratio={Logger.LAST_LOGGED_CLEARING_RATIO}')
        Logger.check_local_enabled()

    @staticmethod
    def initialize(node: Node):
        """
        Initialize the logger instance.

        This method must be called as part of the state machine initialization
        prior to using any logging functions as the use of
        self._ros_logger is unprotected by design.
        """
        if Logger._node is node and Logger._pub is not None:
            Logger._refresh_parameters(node)
            return

        if Logger._node is not None and Logger._pub is not None and Logger._node is not node:
            try:
                Logger._node.destroy_publisher(Logger._pub)
            except Exception:  # pylint: disable=W0703
                pass

        Logger._node = node
        Logger._ros_logger = node.get_logger()
        Logger._pub = node.create_publisher(BehaviorLog, Topics._BEHAVIOR_LOGGING_TOPIC, 100)
        Logger._refresh_parameters(node)

    @staticmethod
    def shutdown():
        """Release the active logger publisher."""
        if Logger._node is not None and Logger._pub is not None:
            try:
                Logger._node.destroy_publisher(Logger._pub)
            except Exception:  # pylint: disable=W0703
                pass
        Logger._pub = None
        Logger._node = None
        Logger._ros_logger = None
        Logger._last_logged = {}
        Logger._local_info_enabled = True
        Logger._local_warn_enabled = True
        Logger._local_hint_enabled = True
        Logger._local_error_enabled = True
        Logger._local_debug_enabled = False

    @staticmethod
    def log(text: str, severity: int):
        """Log message."""
        # send message with logged text
        msg = BehaviorLog()
        msg.text = str(text)
        msg.status_code = severity
        Logger._pub.publish(msg)
        # also log locally
        if Logger._local_enabled(severity):
            Logger.local(text, severity)

    @staticmethod
    def _purge_throttle_cache():
        """Remove the oldest entries from the throttle cache when it exceeds the size limit."""
        if len(Logger._last_logged) > Logger.MAX_LAST_LOGGED_SIZE:
            clear_size = Logger.MAX_LAST_LOGGED_SIZE * (1 - Logger.LAST_LOGGED_CLEARING_RATIO)
            for i, log in enumerate(sorted(Logger._last_logged.items(), key=lambda item: item[1], reverse=True)):
                if i > clear_size:
                    Logger._last_logged.pop(log[0])

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
        Logger._purge_throttle_cache()

    @staticmethod
    def local_throttle(period: float, text: str, severity: int, *args):
        """Locally log unique messages once and don't repeat messages."""
        if not Logger._local_enabled(severity):
            return

        rendered = text % args if args else text
        log_id = f'local_{severity}_{rendered}'
        time_now = Logger._node.get_clock().now()
        if log_id not in Logger._last_logged.keys() or \
           time_now - Logger._last_logged[log_id] > Duration(seconds=period):
            Logger.local(rendered, severity)
            Logger._last_logged.update({log_id: time_now})
        Logger._purge_throttle_cache()

    @staticmethod
    def local(text: str, severity: int):
        """Local logging to terminal."""
        if severity == Logger.REPORT_INFO:
            rcutils_severity = LoggingSeverity.INFO
            rendered = text
        elif severity == Logger.REPORT_WARN:
            rcutils_severity = LoggingSeverity.WARN
            rendered = f'\033[93m{text}\033[0m'
        elif severity == Logger.REPORT_HINT:
            rcutils_severity = LoggingSeverity.INFO
            rendered = f'\033[94mBehavior Hint: {text}\033[0m'
        elif severity == Logger.REPORT_ERROR:
            rcutils_severity = LoggingSeverity.ERROR
            rendered = f'\033[91m{text}\033[0m'
        elif severity == Logger.REPORT_DEBUG:
            rcutils_severity = LoggingSeverity.DEBUG
            rendered = f'\033[92m{text}\033[0m'
        else:
            rcutils_severity = LoggingSeverity.DEBUG
            rendered = f'\033[95m{text}\033[91m(unknown log level {str(severity)})\033[0m'

        _rclpy.rclpy_logging_rcutils_log(
            rcutils_severity,
            Logger._ros_logger.name,
            rendered,
            'flexbe_local',
            __file__,
            1,
        )

    @staticmethod
    def _local_enabled(severity: int):
        """Return whether the local ROS logger is enabled for a given severity."""
        if severity == Logger.REPORT_INFO:
            return Logger._local_info_enabled
        if severity == Logger.REPORT_WARN:
            return Logger._local_warn_enabled
        if severity == Logger.REPORT_HINT:
            return Logger._local_hint_enabled
        if severity == Logger.REPORT_ERROR:
            return Logger._local_error_enabled
        return Logger._local_debug_enabled

    @staticmethod
    def check_local_enabled():
        """Refresh cached local logging enablement flags from the ROS logger."""
        ros_logger = Logger._get_ros_logger()
        Logger._local_info_enabled = ros_logger.is_enabled_for(LoggingSeverity.INFO)
        Logger._local_warn_enabled = ros_logger.is_enabled_for(LoggingSeverity.WARN)
        Logger._local_hint_enabled = ros_logger.is_enabled_for(LoggingSeverity.INFO)
        Logger._local_error_enabled = ros_logger.is_enabled_for(LoggingSeverity.ERROR)
        Logger._local_debug_enabled = ros_logger.is_enabled_for(LoggingSeverity.DEBUG)

    @staticmethod
    def _get_ros_logger():
        """Return the ROS logger for the initialized node."""
        return Logger._node.get_logger()

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
        if Logger._local_debug_enabled:
            Logger.local(text % args if args else text, Logger.REPORT_DEBUG)

    @staticmethod
    def localinfo(text: str, *args):
        """Local info."""
        if Logger._local_info_enabled:
            Logger.local(text % args if args else text, Logger.REPORT_INFO)

    @staticmethod
    def localwarn(text: str, *args):
        """Local warn."""
        if Logger._local_warn_enabled:
            Logger.local(text % args if args else text, Logger.REPORT_WARN)

    @staticmethod
    def localhint(text: str, *args):
        """Local hint."""
        if Logger._local_hint_enabled:
            Logger.local(text % args if args else text, Logger.REPORT_HINT)

    @staticmethod
    def localerr(text: str, *args):
        """Local error."""
        if Logger._local_error_enabled:
            Logger.local(text % args if args else text, Logger.REPORT_ERROR)

    @staticmethod
    def localdebug_throttle(period: float, text: str, *args):
        """Local debug throttle."""
        Logger.local_throttle(period, text, Logger.REPORT_DEBUG, *args)

    @staticmethod
    def localinfo_throttle(period: float, text: str, *args):
        """Local info throttle."""
        Logger.local_throttle(period, text, Logger.REPORT_INFO, *args)

    @staticmethod
    def localwarn_throttle(period: float, text: str, *args):
        """Local warn throttle."""
        Logger.local_throttle(period, text, Logger.REPORT_WARN, *args)

    @staticmethod
    def localhint_throttle(period: float, text: str, *args):
        """Local hint throttle."""
        Logger.local_throttle(period, text, Logger.REPORT_HINT, *args)

    @staticmethod
    def localerr_throttle(period: float, text: str, *args):
        """Local error throttle."""
        Logger.local_throttle(period, text, Logger.REPORT_ERROR, *args)

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
