#!/usr/bin/env python
import traceback
import rclpy
from rclpy.node import Node

class Logger(object):
    """ Bundles static methods for test case logging. """

    _node = None

    @staticmethod
    def initialize(node: Node):
        Logger._node = node

    @classmethod
    def _param_positive(cls):
        print_debug_positive = False
        try:
            print_debug_positive = Logger._node.get_parameter('~print_debug_positive').get_parameter_value()
        except Exception as e:
            Logger._node.declare_parameter('~print_debug_positive', True)
            print_debug_positive = Logger._node.get_parameter('~print_debug_positive').get_parameter_value()

        return not cls._param_compact() and print_debug_positive

    @classmethod
    def _param_negative(cls):
        print_debug_negative = False
        try:
            print_debug_negative = Logger._node.get_parameter('~print_debug_negative').get_parameter_value()
        except Exception as e:
            Logger._node.declare_parameter('~print_debug_negative', True)
            print_debug_positive = Logger._node.get_parameter('~print_debug_negative').get_parameter_value()

        return cls._param_compact() or print_debug_negative

    @classmethod
    def _param_compact(cls):
        compact_format = False
        try:
            compact_format = Logger._node.get_parameter('~compact_format').get_parameter_value()
        except Exception as e:
            Logger._node.declare_parameter('~compact_format', False)
            compact_format = Logger._node.get_parameter('~compact_format').get_parameter_value()

        return compact_format

    @classmethod
    def _prefix(cls):
        return '  >' if cls._param_compact() else '>>>'

    @classmethod
    def _counter(cls):
        cls._counter_value += 1
        return cls._counter_value
    _counter_value = 0

    @classmethod
    def mute_rclpy(cls):
        """ Conditionally mute the rclpy logging channels. """
        mute_info = False
        mute_warn = False
        mute_error = False

        try:
            mute_info = Logger._node.get_parameter('~mute_info').get_parameter_value()
        except Exception as e:
            Logger._node.declare_parameter('~mute_info', False)
            mute_info = Logger._node.get_parameter('~mute_info').get_parameter_value()

        try:
            mute_warn = Logger._node.get_parameter('~mute_warn').get_parameter_value()
        except Exception as e:
            Logger._node.declare_parameter('~mute_warn', False)
            mute_warn = Logger._node.get_parameter('~mute_warn').get_parameter_value()

        try:
            mute_error = Logger._node.get_parameter('~mute_error').get_parameter_value()
        except Exception as e:
            Logger._node.declare_parameter('~mute_error', False)
            mute_error = Logger._node.get_parameter('~mute_error').get_parameter_value()

        if cls._param_compact() or mute_info:
            Logger._node.get_logger().info = Logger._node.get_logger().debug
        if cls._param_compact() or mute_warn:
            Logger._node.get_logger().warn = Logger._node.get_logger().debug
        if not cls._param_compact() and mute_error:
            Logger._node.get_logger().error = Logger._node.get_logger().debug

    @classmethod
    def print_positive(cls, text):
        """ Print a positive intermediate result. """
        if cls._param_positive():
            print('\033[0m\033[1m  +\033[0m %s' % str(text))

    @classmethod
    def print_negative(cls, text):
        """ Print a negative intermediate result. """
        if cls._param_negative():
            print('\033[0m\033[1m  -\033[0m %s' % str(text))

    @classmethod
    def print_title(cls, test_name, test_class, result=None):
        """ Print the title of the test, should be called once and before any other print method. """
        test_result = ' > %s' % result if result is not None else ''
        print('\033[34;1m#%2d %s \033[0m\033[34m(%s%s)\033[0m' % (
            cls._counter(), test_name, test_class, test_result
        ))

    @classmethod
    def print_result(cls, test_name, success):
        """ Print the result, should be called once and after any other print method. """
        test_result = 'completed' if success else 'failed'
        color = '32' if success else '31'
        print('\033[%s;1m%s\033[0m\033[%sm %s %s!\033[0m' % (color, cls._prefix(), color, test_name, test_result))

    @classmethod
    def print_failure(cls, text):
        """ Instead of a result, print the failure of a test case once after any other print method. """
        traceback.print_exc()
        print('\033[31;1m%s\033[0m\033[31m %s\033[0m' % (cls._prefix(), str(text)))

    @classmethod
    def print_error(cls, text):
        """ Print an internal error that might cause unexpected behavior, but does not cause failure itself. """
        print('\033[33;1m   \033[0m\033[33m %s\033[0m' % str(text))

    def __init__(self):
        """ DO NOT USE: use class print methods instead. """
        raise NotImplementedError("use static methods and attributes")
