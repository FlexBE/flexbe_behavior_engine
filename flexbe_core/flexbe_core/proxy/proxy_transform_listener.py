#!/usr/bin/env python
import tf2_ros


#TODO implement tf client
class ProxyTransformListener(object):
    _node = None
    _listener = None

    def _initialize(node):
        ProxyTransformListener._node = node

    def __init__(self):
        if ProxyTransformListener._listener is None:
            tfBuffer = tf2_ros.Buffer()
            ProxyTransformListener._listener = tf2_ros.TransformListener(tfBuffer, ProxyTransformListener._node)

    def listener(self):
        return ProxyTransformListener._listener
