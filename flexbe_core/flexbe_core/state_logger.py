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


"""Logger for active state information."""

from functools import wraps, partial

import logging
import logging.config
import os
import pickle
import time
import yaml

from rclpy.exceptions import ParameterNotDeclaredException

from std_msgs.msg import String

from flexbe_core.proxy import ProxyPublisher


class StateLogger:
    """Realize logging of active states."""

    enabled = False
    _serialize_impl = 'yaml'
    _node = None

    # parameters
    _log_folder = None
    _log_enabled = None
    _log_serialize = None
    _log_level = None
    _log_config = None

    @staticmethod
    def initialize_ros(node):
        StateLogger._node = node
        try:
            StateLogger._log_folder = node.get_parameter('log_folder')
        except ParameterNotDeclaredException:
            StateLogger._log_folder = node.declare_parameter('log_folder', '~/.flexbe_logs')

        try:
            StateLogger._log_enabled = node.get_parameter('log_enabled')
        except ParameterNotDeclaredException:
            StateLogger._log_enabled = node.declare_parameter('log_enabled', False)

        try:
            StateLogger._log_serialize = node.get_parameter('log_serialize')
        except ParameterNotDeclaredException:
            StateLogger._log_serialize = node.declare_parameter('log_serialize', 'yaml')

        try:
            StateLogger._log_level = node.get_parameter('log_level')
        except ParameterNotDeclaredException:
            StateLogger._log_level = node.declare_parameter('log_level', 'INFO')

        try:
            StateLogger._log_config = node.get_parameter('log_config')
        except ParameterNotDeclaredException:
            StateLogger._log_config = node.declare_parameter('log_config', '')

    @staticmethod
    def initialize(be_name=None):
        log_folder = os.path.expanduser(StateLogger._log_folder.get_parameter_value().string_value)

        if log_folder == "" or not StateLogger._log_enabled.get_parameter_value().bool_value:
            StateLogger.enabled = False
            return
        StateLogger.enabled = True

        if not os.path.exists(log_folder):
            os.makedirs(log_folder)

        name = "states"
        if be_name is not None:
            name = be_name.replace(" ", "_").replace(",", "_").replace(".", "_").replace("/", "_").lower()

        StateLogger._serialize_impl = StateLogger._log_serialize.get_parameter_value().string_value

        logger_config = dict({
            'version': 1,
            'disable_existing_loggers': False,
            'formatters': {'yaml': {'()': 'flexbe_core.state_logger.YamlFormatter'}},
            'handlers': {
                'file': {
                    'class': 'logging.FileHandler',
                    'filename': '%(log_folder)s/%(behavior)s_%(timestamp)s.yaml',
                    'formatter': 'yaml'
                },
                'publish': {
                    'class': 'flexbe_core.state_logger.PublishBehaviorLogMessage',
                    'topic': 'flexbe/state_logger',
                    'formatter': 'yaml'
                }
            },
            'loggers': {'flexbe': {'level': 'INFO', 'handlers': ['file']}}
        }, **yaml.safe_load(StateLogger._log_config.get_parameter_value().string_value))
        if ('handlers' in logger_config and 'file' in logger_config['handlers']
                and 'filename' in logger_config['handlers']['file']):
            logger_config['handlers']['file']['filename'] %= {
                'log_folder': log_folder,
                'behavior': name,
                'timestamp': time.strftime("%Y-%m-%d-%H_%M_%S")
            }
        if 'loggers' in logger_config and 'flexbe' in logger_config['loggers']:
            level = StateLogger._log_level.get_parameter_value().string_value
            logger_config['loggers']['flexbe']['level'] = level.upper()
        logging.config.dictConfig(logger_config)

    @staticmethod
    def shutdown():
        if not StateLogger.enabled:
            return
        logging.shutdown()
        StateLogger.enabled = False

    @staticmethod
    def get(name):
        """Obtain a reference to the named logger."""
        return logging.getLogger(name)

    @staticmethod
    def log(name, state, **kwargs):
        """Log custom data as given by the keyword arguments."""
        if StateLogger.enabled:
            StateLogger.get(name).log(kwargs.get('loglevel', logging.INFO), dict(StateLogger._basic(state), **kwargs))

    # state decorators

    @staticmethod
    def log_events(name, **events):
        """Log whenever any of the specified events of the state is activated."""
        def decorator(cls):
            cls_init = cls.__init__

            @wraps(cls.__init__)
            def log_events_init(self, *args, **kwargs):
                cls_init(self, *args, **kwargs)
                for event, method in events.items():
                    def wrap_event_method(event, method):
                        if hasattr(self, method):
                            event_method = getattr(self, method)

                            @wraps(event_method)
                            def event_wrapper(*args, **kwargs):
                                time_start = StateLogger._node.get_clock().now().nanoseconds
                                try:
                                    event_method(*args, **kwargs)
                                finally:
                                    if StateLogger.enabled:
                                        duration = StateLogger._node.get_clock().now().nanoseconds - time_start
                                        StateLogger.get(name).info(dict(
                                            StateLogger._basic(self),
                                            event=event,
                                            duration=duration * 1e-9))
                            setattr(self, method, event_wrapper)
                    wrap_event_method(event, method)
            cls.__init__ = log_events_init
            return cls
        return decorator

    @staticmethod
    def log_outcomes(name):
        """Log all outcomes of the state."""
        def decorator(cls):
            cls_init = cls.__init__

            @wraps(cls.__init__)
            def log_outcomes_init(self, *args, **kwargs):
                cls_init(self, *args, **kwargs)
                execute_method = getattr(self, 'execute')

                @wraps(execute_method)
                def execute_wrapper(*args, **kwargs):
                    outcome = None
                    try:
                        outcome = execute_method(*args, **kwargs)
                        return outcome
                    finally:
                        if StateLogger.enabled and outcome is not None:
                            StateLogger.get(name).info(dict(
                                StateLogger._basic(self),
                                outcome=outcome))
                setattr(self, 'execute', execute_wrapper)
            cls.__init__ = log_outcomes_init
            return cls
        return decorator

    @staticmethod
    def log_userdata(name, keys=None):
        """Log all userdata that is passed to the state."""
        def decorator(cls):
            cls_init = cls.__init__

            @wraps(cls.__init__)
            def log_userdata_init(self, *args, **kwargs):
                cls_init(self, *args, **kwargs)
                input_keys = kwargs.get('input_keys', [])
                on_enter_method = getattr(self, 'on_enter')

                @wraps(on_enter_method)
                def on_enter_wrapper(userdata):
                    logger = StateLogger.get(name)
                    if StateLogger.enabled and logger.isEnabledFor(logging.DEBUG) and input_keys:
                        logdata = dict(StateLogger._basic(self))
                        logdata['userdata'] = {}
                        for key in input_keys:
                            if keys is not None and key not in keys:
                                continue
                            try:
                                logdata['userdata'][key] = StateLogger._serialize(userdata[key])
                            except Exception as exc:  # pylint: disable=W0703
                                from flexbe_core import Logger  # pylint: disable=C0415
                                Logger.warning('State %s failed to log userdata for key %s: %s' %
                                               (self.name, key, str(exc)))
                        logger.debug(logdata)
                    on_enter_method(userdata)
                setattr(self, 'on_enter', on_enter_wrapper)
            cls.__init__ = log_userdata_init
            return cls
        return decorator

    # helpers

    @staticmethod
    def _serialize(obj):
        return {
            'yaml': partial(yaml.dump, default_flow_style=True),
            'str': str,
            'repr': repr,
            'pickle': pickle.dumps,
        }.get(StateLogger._serialize_impl, lambda o: eval(StateLogger._serialize_impl,  # pylint: disable=W0123
                                                          locals={'object': o}))(obj)

    @staticmethod
    def _basic(state):
        result = {'time': StateLogger._node.get_clock().now().nanoseconds * 1e-9}
        if state is not None:
            result.update({
                'name': state.name,
                'state': state.__class__.__name__,
                'path': state.path
            })
        return result


class YamlFormatter(logging.Formatter):
    """Special yaml formatting class."""

    def format(self, record):
        record.msg.update(logger=record.name, loglevel=record.levelname)
        return '- %s' % super().format(record)


class PublishBehaviorLogMessage(logging.Handler):
    """publish messages to behavior logs."""

    def __init__(self, level=logging.NOTSET, topic='flexbe/state_logger'):
        super().__init__(level)
        self._topic = topic
        self._pub = ProxyPublisher({self._topic: String})

    def emit(self, record):
        message = self.format(record)
        self._pub.publish(self._topic, String(data=message))
