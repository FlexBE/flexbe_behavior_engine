#!/usr/bin/env python
import os
import time
import yaml
import pickle
import logging
import logging.config
from functools import wraps, partial
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String
from rclpy.exceptions import ParameterNotDeclaredException


class StateLogger(object):
    '''
    Realizes logging of active states.
    '''
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
        except ParameterNotDeclaredException as e:
            StateLogger._log_folder = node.declare_parameter('log_folder', '~/.flexbe_logs')

        try:
            StateLogger._log_enabled = node.get_parameter('log_enabled')
        except ParameterNotDeclaredException as e:
            StateLogger._log_enabled = node.declare_parameter('log_enabled', False)

        try:
            StateLogger._log_serialize = node.get_parameter('log_serialize')
        except ParameterNotDeclaredException as e:
            StateLogger._log_serialize = node.declare_parameter('log_serialize', 'yaml')

        try:
            StateLogger._log_level = node.get_parameter('log_level')
        except ParameterNotDeclaredException as e:
            StateLogger._log_level = node.declare_parameter('log_level', 'INFO')

        try:
            StateLogger._log_config = node.get_parameter('log_config')
        except ParameterNotDeclaredException as e:
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
        if ('handlers' in logger_config and 'file' in logger_config['handlers'] and
                'filename' in logger_config['handlers']['file']):
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
        """ Obtain a reference to the named logger. """
        return logging.getLogger(name)

    @staticmethod
    def log(name, state, **kwargs):
        """ Log custom data as given by the keyword arguments. """
        if StateLogger.enabled:
            StateLogger.get(name).log(kwargs.get('loglevel', logging.INFO), dict(StateLogger._basic(state), **kwargs))

    # state decorators

    @staticmethod
    def log_events(name, **events):
        """ Log whenever any of the specified events of the state is activated. """
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
        """ Log all outcomes of the state. """
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
        """ Log all userdata that is passed to the state. """
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
                        logdata = dict(StateLogger._basic(self), userdata=dict())
                        for key in input_keys:
                            if keys is not None and key not in keys:
                                continue
                            try:
                                logdata['userdata'][key] = StateLogger._serialize(userdata[key])
                            except Exception as e:
                                from flexbe_core import Logger
                                Logger.warning('State %s failed to log userdata for key %s: %s' %
                                               (self.name, key, str(e)))
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
        }.get(StateLogger._serialize_impl, lambda o: eval(StateLogger._serialize_impl, locals={'object': o}))(obj)

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

    def format(self, record):
        record.msg.update(logger=record.name, loglevel=record.levelname)
        return '- %s' % super(YamlFormatter, self).format(record)


class PublishBehaviorLogMessage(logging.Handler):

    def __init__(self, level=logging.NOTSET, topic='flexbe/state_logger'):
        super(PublishBehaviorLogMessage, self).__init__(level)
        self._topic = topic
        self._pub = ProxyPublisher({self._topic: String})

    def emit(self, record):
        message = self.format(record)
        self._pub.publish(self._topic, String(message))
