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


"""Provides an interface for required test case data."""

import os
# import rosbag
from ament_index_python.packages import get_package_share_directory
from .logger import Logger


class DataProvider:
    """Provides an interface for required test case data."""

    __test__ = False  # Do not pytest this class (it is the test!)

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

            # storage_options, converter_options = get_rosbag_options(bagpath)
            raise NotImplementedError("ROS bag loading not implemented in ROS 2 - TODO!")

            # self._bag = rosbag2_py.SequentialReader()
            # self._bag.open(storage_options, converter_options)

            # self._bag = rosbag.Bag(bagpath)
            Logger.print_positive('using data source: %s' % bagpath)

    def parse(self, value):
        """Replace special values according to the specification."""
        result = value
        try:
            # message data
            if (isinstance(value, str) and len(value) > 1 and value[0] == '/' and value[1] != '/'
                    and self._bag is not None):
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
