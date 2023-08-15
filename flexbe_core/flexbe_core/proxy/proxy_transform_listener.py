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


"""Provide a single tf listener per node for all states that need tf operations."""

import tf2_ros
from flexbe_core.logger import Logger


class ProxyTransformListener:
    """Provide a single tf listener per node for all states that need tf operations."""

    _node = None
    _listener = None
    _buffer = None

    @staticmethod
    def initialize(node):
        """Initialize with ROS info."""
        ProxyTransformListener._node = node
        Logger.initialize(node)

    @staticmethod
    def shutdown():
        """Shut down this proxy by reseting the transform listener."""
        try:
            ProxyTransformListener._listener = None
            ProxyTransformListener._buffer = None
            print("Shutdown proxy transform listener - finished!")
        except Exception as exc:  # pylint: disable=W0703
            print(f'Something went wrong during shutdown of proxy transform listener  !\n{str(exc)}')

    def __init__(self):
        """Only start listening to tf if someone creates an instance of this listener."""
        if ProxyTransformListener._listener is None:
            ProxyTransformListener._buffer = tf2_ros.Buffer()
            ProxyTransformListener._listener = tf2_ros.TransformListener(ProxyTransformListener._buffer,
                                                                         ProxyTransformListener._node)

    @property
    def listener(self):
        """Return listener instance."""
        return ProxyTransformListener._listener

    @property
    def buffer(self):
        """Return the TF buffer instance to allow calling transforms."""
        return ProxyTransformListener._buffer
