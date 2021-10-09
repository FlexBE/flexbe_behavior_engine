#!/usr/bin/env python
import genmsg
import importlib
from rclpy.duration import Duration
from rclpy.time import Time

STANDARD_MSG_TYPES = ['bool', 'int8', 'uint8', 'int16', 'uint16', 'int32',
                      'uint32', 'int64', 'uint64', 'float32', 'float64',
                      'string', 'char','byte', 'octet']
_message_class_cache = {}

def fill_message_args(msg, msg_args, keys={}):
    """
    Populate message with specified args.

    Args are assumed to be a
    list of arguments from a command-line YAML parser. See
    http://www.ros.org/wiki/ROS/YAMLCommandLine for specification on
    how messages are filled.

    fill_message_args also takes in an optional 'keys' dictionary
    which contain substitute values for message and time types. These
    values must be of the correct instance type, i.e. a Message, Time,
    or Duration. In a string key is encountered with these types, the
    value from the keys dictionary will be used instead. This is
    mainly used to provide values for the 'now' timestamp.

    :param msg: message to fill, ``Message``
    :param msg_args: list of arguments to set fields to, or
      If None, msg_args will be made an empty list., ``[args]``
    :param keys: keys to use as substitute values for messages and timestamps, ``dict``
    :raises: :exc:`ValueError` If not enough/too many message arguments to fill message
    """

    # msg_args is always a list, due to the fact it is parsed from a
    # command-line argument list.  We have to special-case handle a
    # list with a single dictionary, which has precedence over the
    # general list representation. We offer this precedence as there
    # is no other way to do kwd assignments into the outer message.

    _fill_message_args(msg, msg_args, keys, '')

def _fill_message_args(msg, msg_args, keys, prefix=''):
    """
    Populate message with specified args.

    :param msg: message to fill, ``Message``
    :param msg_args: list of arguments to set fields to, ``[args]``
    :param keys: keys to use as substitute values for messages and timestamps.  ``dict``
    :param prefix: field name prefix (for verbose printing), ``str``
    :returns: unused/leftover message arguments.  ``[args]``
    :raise :exc:`ValueError` If not enough message arguments to fill message
    :raises: :exc:`ValueError` If msg or msg_args is not of correct type
    """
    if type(msg_args) == dict:
        for field, val in msg_args.items():
            # assume that an empty key is actually an empty string
            if val is None:
                val = ''
            _fill_val(msg, field, val, keys, prefix)
    elif type(msg_args) == list:
        if len(msg_args) > len(msg.get_fields_and_field_types()):
            raise ValueError('Too many arguments:\n * Given: %s\n * Expected: %s' % (msg_args, msg.get_fields_and_field_types()))
        elif len(msg_args) < len(msg.get_fields_and_field_types()):
            raise ValueError('Not enough arguments:\n * Given: %s\n * Expected: %s' % (msg_args, msg.get_fields_and_field_types()))

        for field in msg_args:
            _fill_val(msg, field, msg_args[field], keys, prefix)
    else:
        raise ValueError('invalid msg_args type: %s' % str(msg_args))

def _fill_val(msg, field, val, keys, prefix):
    """
    Subroutine of L{_fill_message_args()}.

    Sets a particular field on a message
    :param field: field name, ``str``
    :param val: field value
    :param keys: keys to use as substitute values for messages and timestamps, ``dict``
    :raises: exc:`ValueError`
    """
    msg_fields_and_field_types = msg.get_fields_and_field_types()
    if field not in msg_fields_and_field_types:
        print(msg_fields_and_field_types)
        raise ValueError('No field name [%s%s]' % (prefix, field))

    def_val = getattr(msg, field)


    if type(def_val) == list:
        if not type(val) in [list, tuple]:
            raise ValueError('Field [%s%s] must be a list or tuple instead of: %s' % (prefix, field, type(val).__name__))
        # determine base_type of field by looking at _slot_types

        t = msg_fields_and_field_types[field]
        base_type, is_array, length = parse_type(t)
        # - for primitives, we just directly set (we don't
        #   type-check. we rely on serialization type checker)
        if base_type in STANDARD_MSG_TYPES:
            byte_list = []
            if base_type == 'octet':
                for i in val:
                    byte_list.append(str(i).encode())
                val = set(byte_list)
            # 3785
            if length is not None and len(val) != length:
                raise ValueError('Field [%s%s] has incorrect number of elements: %s != %s' % (prefix, field, len(val), length))
            setattr(msg, field, val)

        # - for complex types, we have to iteratively append to def_val
        else:
            # 3785
            if length is not None and len(val) != length:
                raise ValueError('Field [%s%s] has incorrect number of elements: %s != %s' % (prefix, field, len(val), length))

            list_msg_class = get_message_class(base_type)
            if list_msg_class is None:
                raise ValueError('Cannot instantiate messages for field [%s%s] : cannot load class %s' % (prefix, field, base_type))
            del def_val[:]
            for el in val:
                inner_msg = list_msg_class()

                if isinstance(inner_msg, Time) and type(el) in (int, long):
                    setattr(msg, field, Time.from_sec(el/1e9))
                elif isinstance(inner_msg, Duration) and type(el) in (int, long):
                    setattr(msg, field, Duration.from_sec(el/1e9))
                else:
                    _fill_message_args(inner_msg, el, keys, prefix)

                def_val.append(inner_msg)
    elif msg_fields_and_field_types[field] not in STANDARD_MSG_TYPES:
        # check for substitution key, e.g. 'now'
        if type(val) == str:
            if val in keys:
                setattr(msg, field, keys[val])
            else:
                raise ValueError('No key named [%s]' % (val))
        elif isinstance(def_val, Time) and type(val) in (int, long):
            setattr(msg, field, Time.from_sec(val/1e9))
        elif isinstance(def_val, Duration) and type(val) in (int, long):
            setattr(msg, field, Duration.from_sec(val/1e9))
        else:
            _fill_message_args(def_val, val, keys, prefix=(prefix+field+'.'))
    else:
        setattr(msg, field, val)

def parse_type(msg_type):
    """
    Parse ROS message field type
    :param msg_type: ROS field type, ``str``
    :returns: base_type, is_array, array_length, ``(str, bool, int)``
    :raises: :exc:`ValueError` If *msg_type* cannot be parsed
    """
    if not msg_type:
        raise ValueError("Invalid empty type")
    if "sequence" in msg_type:
        if '<' in msg_type and '>' in msg_type:
            # raise ValueError("Sequence type " + msg_type[msg_type.index('<') + 1:msg_type.index('>')])
            return msg_type[msg_type.index('<') + 1:msg_type.index('>')], True, None

        var_length = msg_type.endswith('[]')
        splits = msg_type.split('[')
        if len(splits) > 2:
            raise ValueError("Currently only support 1-dimensional array types: %s"%msg_type)
        if var_length:
            return msg_type[:-2], True, None
        else:
            try:
                length = int(splits[1][:-1])
                return splits[0], True, length
            except ValueError:
                raise ValueError("Invalid array dimension: [%s]"%splits[1][:-1])
    else:
        return msg_type, False, None

def get_message_class(message_type, reload_on_error=False):
    """
    Get the message class.

    NOTE: this function maintains a local cache of results to improve
    performance.
    :param message_type: type name of message, ``str``
    :param reload_on_error: (optional). Attempt to reload the Python
      module if unable to load message the first time. Defaults to
      False. This is necessary if messages are built after the first load.
    :returns: Message class for message/service type, ``Message class``
    :raises :exc:`ValueError`: if  message_type is invalidly specified
    """
    if message_type in _message_class_cache:
        return _message_class_cache[message_type]
    cls = _get_message_class('msg', message_type, reload_on_error=reload_on_error)
    if cls:
        _message_class_cache[message_type] = cls
    return cls

def _get_message_class(type_str, message_type, reload_on_error=False):
    """
    Retrieve message/service class instances.

    Used by get_message_class and get_service_class.
    :param type_str: 'msg' or 'srv', ``str``
    :param message_type: type name of message/service, ``str``
    :returns: Message/Service  for message/service type or None, ``class``
    :raises: :exc:`ValueError` If message_type is invalidly specified
    """
    if message_type == 'time':
        return Time
    if message_type == 'duration':
        return Duration
    # parse package and local type name for import
    package, base_type = genmsg.package_resource_name(message_type)

    if not package:
        if base_type == 'Header':
            package = 'std_msgs'
        else:
            raise ValueError('message type is missing package name: %s' % str(message_type))
    pypkg = val = None
    try:
        # import the package
        pypkg = __import__('%s.%s' % (package, type_str))
    except ImportError:
        pass
    if pypkg:
        try:
            val = getattr(getattr(pypkg, type_str), base_type)
        except AttributeError:
            pass

    # this logic is mainly to support rosh, so that a user doesn't
    # have to exit a shell just because a message wasn't built yet
    if val is None and reload_on_error:
        try:
            if pypkg:
                reload(pypkg)
            val = getattr(getattr(pypkg, type_str), base_type)
        except Exception:
            val = None
    return val
