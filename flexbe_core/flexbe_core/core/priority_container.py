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


"""A state machine that is always executed alone when becoming active."""
from flexbe_core.core.operatable_state_machine import OperatableStateMachine
from flexbe_core.logger import Logger


class PriorityContainer(OperatableStateMachine):
    """A state machine that is always executed alone when becoming active."""

    active_container = None

    def __init__(self, conditions=None, *args, **kwargs):
        """Initialize instance of PriorityContainer."""
        super().__init__(*args, **kwargs)
        self._parent_active_container = None
        self._type = OperatableStateMachine.ContainerType.PriorityContainer.value

    def execute(self, *args, **kwargs):
        """Execute the priority container."""
        if (PriorityContainer.active_container is None
            or not all(p == PriorityContainer.active_container.split('/')[i]
                       for i, p in enumerate(self.path.split('/')))):
            self._parent_active_container = PriorityContainer.active_container
            PriorityContainer.active_container = self.path

        outcome = OperatableStateMachine.execute(self, *args, **kwargs)

        if outcome is not None:
            # Logger.localinfo(f"Priority container '{self}' outcome='{outcome}' reset active "
            #                  f"from '{PriorityContainer.active_container}'"
            #                  f" to  '{self._parent_active_container}'")
            PriorityContainer.active_container = self._parent_active_container

        return outcome

    def on_enter(self, userdata=None):  # pylint: disable=W0613
        """Call on entering the operatable state machine."""
        Logger.localinfo(f"Enter priority container '{self}' ...")
        super().on_enter(userdata)

    def on_exit(self, userdata=None):
        """Call on exiting the statemachine."""
        super().on_enter(userdata)
        Logger.localinfo(f"Exited priority container '{self}'")
