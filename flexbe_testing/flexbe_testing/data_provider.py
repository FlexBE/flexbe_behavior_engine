#!/usr/bin/env python
import os
# import rosbag
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from .logger import Logger


class DataProvider(object):
    """ Provides an interface for required test case data. """

    def __init__(self, node, bagfile=None):
        self.node = node
        Logger.initialize(node)
        self._bag = None

        if bagfile is not None:
            bagpath = ''
            # absolute path
            if bagfile.startswith('~') or bagfile.startswith('/'):
                bagpath = os.path.expanduser(bagfile)
            # package-relative path
            else:
                pkgpath = get_package_share_directory(bagfile.split('/')[0])
                bagpath = os.path.join(pkgpath, '/'.join(bagfile.split('/')[1:]))

            storage_options, converter_options = get_rosbag_options(bagpath)

            # self._bag = rosbag2_py.SequentialReader()
            # self._bag.open(storage_options, converter_options)

            # self._bag = rosbag.Bag(bagpath)
            Logger.print_positive('using data source: %s' % bagpath)

    def parse(self, value):
        """ Replace special values according to the specification. """
        result = value
        try:
            # message data
            if (isinstance(value, str) and len(value) > 1 and value[0] == '/' and value[1] != '/' and
                    self._bag is not None):
                try:
                    (_, result, _) = self._bag.read_next()
                    # (_, result, _) = list(self._bag.read_messages(topics=[value]))[0]
                except Exception as e:
                    Logger.print_error('Error parsing %s' % (str(e)))
                    # (_, result, _) = list(self._bag.read_messages(topics=[value[1:]]))[0]
            # anonymous function
            elif isinstance(value, str) and value.startswith('lambda '):
                result = eval(value)
            # None
            elif value == 'None':
                result = None
            # escaped backslash at the beginning
            elif isinstance(value, str) and len(value) > 1 and value[0] == '/' and value[1] == '/':
                result = value[1:]
        except Exception as e:
            Logger.print_error('unable to parse value "%s" (will be considered as string):\n\t%s' % (
                str(value), str(e)
            ))
            result = str(value)
        return result
