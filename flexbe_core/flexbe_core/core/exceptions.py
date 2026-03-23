#!/usr/bin/env python3

# Copyright 2024 Philipp Schillinger, Team ViGIR, Christopher Newport University
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


"""FlexBE Core Exceptions."""


class StateError(Exception):
    """State Error."""


class StateMachineError(Exception):
    """StateMachine Error."""


class UserDataError(Exception):
    """UserData Error."""


class FlexBEError(Exception):
    """Base class for FlexBE domain-specific errors."""


class ProxyError(FlexBEError):
    """Base class for proxy-layer errors."""


class ProxyAvailabilityError(ValueError, ProxyError):
    """Raised when a proxy operation cannot proceed due to unavailable resources."""


class ProxyTypeError(TypeError, ProxyError):
    """Raised when proxy payload or interface types are invalid."""


class TransitionError(FlexBEError):
    """Raised for invalid state transition or transition handling failures."""


class SyncError(FlexBEError):
    """Raised when distributed state synchronization fails."""


class BehaviorLoadError(FlexBEError):
    """Raised when loading/preparing a behavior fails."""


class ShutdownError(FlexBEError):
    """Raised when shutdown/cleanup operations fail."""
