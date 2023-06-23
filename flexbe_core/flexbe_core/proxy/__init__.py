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


"""Initialize of the flexbe_core.proxy module."""

from .proxy_action_client import ProxyActionClient  # noqa: F401
from .proxy_publisher import ProxyPublisher  # noqa: F401
from .proxy_service_caller import ProxyServiceCaller  # noqa: F401
from .proxy_subscriber_cached import ProxySubscriberCached  # noqa: F401
from .proxy_transform_listener import ProxyTransformListener  # noqa: F401


def initialize_proxies(node):
    """Initialize all proxies given node instance."""
    ProxySubscriberCached.initialize(node)
    ProxyPublisher.initialize(node)
    ProxyServiceCaller.initialize(node)
    ProxyTransformListener.initialize(node)
    ProxyActionClient.initialize(node)


def shutdown_proxies():
    """
    Shutdown all proxies.

    This should only be called after stopping and shutting down all behaviors
    and the behavior engine.

    Mostly used during testing
    """
    ProxySubscriberCached.shutdown()
    ProxyPublisher.shutdown()
    ProxyServiceCaller.shutdown()
    ProxyTransformListener.shutdown()
    ProxyActionClient.shutdown()


__all__ = [
    'ProxyActionClient',
    'ProxyPublisher',
    'ProxyServiceCaller',
    'ProxySubscriberCached',
    'ProxyTransformListener',
    'initialize_proxies',
    'shutdown_proxies'
]
