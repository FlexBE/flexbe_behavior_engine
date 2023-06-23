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

"""
Initialization for flexbe_core.core module.

Please use EventState as parent class for new states
because it extends all other parent classes.

For a behavior, inherit from OperatableStateMachine as state machine.

"""
from .core import ConcurrencyContainer, EventState   # noqa: F401
from .core import OperatableStateMachine, PriorityContainer  # noqa: F401

from .behavior import Behavior  # noqa: F401

from .behavior_library import BehaviorLibrary  # noqa: F401

from .logger import Logger  # noqa: F401
from .state_logger import StateLogger  # noqa: F401

# pylint: disable=R0903


def set_node(node):
    """Set node information and initialize classes."""
    from .proxy import initialize_proxies        # pylint: disable=C0415
    from .core import RosState, RosStateMachine  # pylint: disable=C0415
    Logger.initialize(node)
    StateLogger.initialize_ros(node)
    initialize_proxies(node)
    RosState.initialize_ros(node)
    RosStateMachine.initialize_ros(node)


class Autonomy:
    """Provides constants for the available required Autonomy Levels."""

    Inherit = 0
    """
    Use this whenever you want to rely on an already defined level of autonomy
    Typical situations: Outcomes of a contained state machine or behavior, outcome of the input request state
    """

    Off = 0
    """
    Use this when no level of autonomy is required, the Autonomy Level needs to be at least 'Off'.
    Typical situations: Outcomes that report internal software errors or thrown exceptions.
    """

    Low = 1
    """
    Use this for reliable decisions that only need validation in the 'low' autonomy mode.
    """

    High = 2
    """
    Use this for more important or high level decisions that will only be executed autonomously
    when in full autonomy and need validation in the 'high' autonomy mode.
    """

    Full = 3
    """
    Use this for outcomes that always need an operator input.
    A use of this level is not recommended.
    """


__all__ = [
    'Behavior',
    'BehaviorLibrary',
    'ConcurrencyContainer',
    'EventState',
    'OperatableStateMachine',
    'PriorityContainer',
    'Logger',
    'StateLogger',
    'set_node',
    'Autonomy'
]
