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


"""Realize behavior-specific logging."""

from rclpy.node import Node

from flexbe_msgs.msg import BehaviorLog


class Logger:
    """Realize behavior-specific logging."""

    REPORT_INFO = BehaviorLog.INFO
    REPORT_WARN = BehaviorLog.WARN
    REPORT_HINT = BehaviorLog.HINT
    REPORT_ERROR = BehaviorLog.ERROR
    REPORT_DEBUG = BehaviorLog.DEBUG

    LOGGING_TOPIC = 'flexbe/log'

    _pub = None
    _node = None

    @staticmethod
    def initialize(node: Node):
        Logger._node = node
        Logger._pub = node.create_publisher(BehaviorLog, Logger.LOGGING_TOPIC, 100)

    @staticmethod
    def log(text: str, severity: int):
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
    def local(text: str, severity: int):
        if Logger._node is None:
            raise RuntimeError('Unable to log, run "Logger.initialize" first to define the target ROS node.')
        if severity == Logger.REPORT_INFO:
            Logger._node.get_logger().info(text)
        elif severity == Logger.REPORT_WARN:
            Logger._node.get_logger().warning(text)
        elif severity == Logger.REPORT_HINT:
            Logger._node.get_logger().info('\033[94mBehavior Hint: %s\033[0m' % text)
        elif severity == Logger.REPORT_ERROR:
            Logger._node.get_logger().error(text)
        elif severity == Logger.REPORT_DEBUG:
            Logger._node.get_logger().debug(text)
        else:
            Logger._node.get_logger().debug(text + ' (unknown log level %s)' % str(severity))

    # NOTE: Below text strings can only have single % symbols if they are being treated
    # as format strings with appropriate arguments (otherwise replace with %% for simple string without args)
    @staticmethod
    def logdebug(text, *args):
        Logger.log(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def loginfo(text, *args):
        Logger.log(text % args, Logger.REPORT_INFO)

    @staticmethod
    def logwarn(text, *args):
        Logger.log(text % args, Logger.REPORT_WARN)

    @staticmethod
    def loghint(text, *args):
        Logger.log(text % args, Logger.REPORT_HINT)

    @staticmethod
    def logerr(text, *args):
        Logger.log(text % args, Logger.REPORT_ERROR)

    @staticmethod
    def localdebug(text, *args):
        Logger.local(text % args, Logger.REPORT_DEBUG)

    @staticmethod
    def localinfo(text, *args):
        Logger.local(text % args, Logger.REPORT_INFO)

    @staticmethod
    def localwarn(text, *args):
        Logger.local(text % args, Logger.REPORT_WARN)

    @staticmethod
    def localerr(text, *args):
        Logger.local(text % args, Logger.REPORT_ERROR)

    @staticmethod
    def debug(text, *args):
        Logger.logdebug(text, *args)

    @staticmethod
    def info(text, *args):
        Logger.loginfo(text, *args)

    @staticmethod
    def warning(text, *args):
        Logger.logwarn(text, *args)

    @staticmethod
    def hint(text, *args):
        Logger.loghint(text, *args)

    @staticmethod
    def error(text, *args):
        Logger.logerr(text, *args)
