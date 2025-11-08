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


"""This defines the superclass for all implemented behaviors."""
from flexbe_core.core import LockableStateMachine, OperatableStateMachine, PreemptableState, StateMap
from flexbe_core.logger import Logger

from flexbe_msgs.msg import BehaviorSync


class Behavior:
    """This is the superclass for all implemented behaviors."""

    def __init__(self):
        """Call this superclass constructor first when overriding it with your behavior."""
        self._state_machine = None
        self.name = 'unnamed behavior'
        self.beh_id = 0  # Behavior id checksum assigned by processing the file contents
        self._state_map = None

        self.contains = {}
        self._behaviors = {}

        self._autonomy_level = 3
        self._debug = False

        self.requested_state_id = None

    # Please implement this abstract method:
    def create(self):
        """
        Create the state machine for this behavior and return it.

        It is called immediately before executing the behavior,
        so used parameters will have their final value when called.

        @return The complete state machine for this behavior.
        """

    # Use those if you need them:
    def add_parameter(self, name, default):
        """
        Add a parameter to this behavior.

        The parameter should be declared in the behavior manifest.

        @type name: string
        @param name: The name of the parameter.

        @type default: object
        @param default: The default value of this parameter. Be sure to set it to the right type.
        """
        setattr(self, name, default)

    def add_behavior(self, behavior_class, behavior_id, node):
        """
        Add another behavior as part of this behavior.

        This other behavior should be declared as contained in the behavior manifest.

        @type behavior_class: class
        @param behavior_class: The class implementing the other behavior.

        @type behavior_id: string
        @param behavior_id: Unique identifier for this behavior instance.
        """
        if not hasattr(self, 'contains'):
            Logger.logerr('Behavior was not initialized! Please call superclass constructor.')
        instance = behavior_class(node)
        self.contains[behavior_id] = instance

    def use_behavior(self, behavior_class, behavior_id, default_keys=None, parameters=None):
        """
        Create a state machine implementing the given behavior to use it in the behavior state machine.

        Behavior has to be added first.

        @type behavior_class: class
        @param behavior_class: The class implementing the other behavior.

        @type behavior_id: string
        @param behavior_id: Same identifier as used for adding.

        @type default_keys: list
        @param default_keys: List of input keys of the behavior which should be ignored
                             and instead use the default values as given by the behavior.

        @type parameters: dict
        @param parameters: Optional assignment of values to behavior parameters.
                           Any assigned parameter will be ignored for runtime customization,
                           i.e., cannot be overwritten by a user who runs the behavior.
        """
        if behavior_id not in self.contains:
            Logger.logerr('Tried to use a behavior without adding it!')
            return None

        if parameters is not None:
            for parameter, value in parameters.items():
                setattr(self.contains[behavior_id], parameter, value)

        state_machine = self.contains[behavior_id]._get_state_machine()

        if default_keys is not None:
            state_machine._input_keys = list(set(state_machine._input_keys) - set(default_keys))
        for key in state_machine._input_keys:
            state_machine._own_userdata(remove_key=key)

        return state_machine

    # Lifecycle

    def prepare_for_execution(self, input_data=None):
        """Prepare this behavior for execution by building its state machine."""
        OperatableStateMachine.autonomy_level = self._autonomy_level

        self._state_machine = self.create()  # pylint: disable=E1111

        if input_data is None:
            input_data = {}
        for key, val in input_data.items():
            if key in self._state_machine._own_userdata:
                self._state_machine._own_userdata[key] = val

    def set_parameter(self, name, value):
        """
        Set the given parameter of this behavior or any sub-behavior.

        Use a path specification to refer to the corresponding sub-behavior.
        The parameter value is propagated to all sub-behaviors according to the propagation rules.
        Also, the value is casted to the type of the parameter if required.

        @type name: string
        @param name: Name of the parameter, possibly including the path to a sub-behavior.

        @type value: object
        @param value: New value of the parameter of same type as before (or can be casted to that type).

        @return: Whether the parameter existed and could be set.
        """
        name_split = name.rsplit('/', 1)
        behavior = name_split[0] if len(name_split) == 2 else ''
        key = name_split[-1]
        behaviors = self.get_contained_behaviors()
        behaviors[''] = self  # add this behavior as well
        found = False
        for beh in behaviors:
            if beh.startswith(behavior) and hasattr(behaviors[beh], key):
                behaviors[beh]._set_typed_attribute(key, value)
                found = True
        return found

    def confirm(self):
        """Confirm that this behavior is ready for execution."""
        self._state_map = StateMap()
        self._state_machine.confirm(self.name, self.beh_id, self._state_map)
        LockableStateMachine.path_for_switch = None
        if self.requested_state_id is not None:
            requested_state = self._state_map[self.requested_state_id]
            LockableStateMachine.path_for_switch = requested_state.path

    @property
    def state_map_items(self):
        """Return two lists of keys and values from state map."""
        if self._state_machine is not None:
            return list(zip(*self._state_map.items))
        return [], []

    def get_state_by_id(self, st_id):
        """Return state reference from state map by id."""
        if self._state_map is not None:
            return self._state_map.get(st_id)
        return None

    def execute(self):
        """
        Execute the behavior.

        Need to call self.execute_behavior when ready to start the state machine and return its result.

        @return: A string containing the execution result such as finished or failed.
        """
        result = self._state_machine.spin()
        self._state_machine.destroy()
        return result

    def get_latest_status(self):
        """Return the latest execution information as a BehaviorSync message."""
        if self._state_machine:
            return self._state_machine.get_latest_status()

        return BehaviorSync()

    def prepare_for_switch(self, state):
        """
        Prepare the behavior for being executed after a behavior switch.

        @type name: string
        @param name: The name of this behavior.
        """
        state._locked = True  # make sure the state cannot transition during preparations
        states = self._get_states_of_path(state.path, self._state_machine)
        if states is None:
            raise RuntimeError('Did not find locked state in new behavior!')
        state_container = state._parent
        state_container.remove_state(state)  # remove from old state machine
        for sm in states[1:]:
            # update userdata in new state machine
            sm.replace_userdata(state_container.userdata)
            state_container = state_container._parent
        states[1].replace_state(state)  # add to new state machine
        self.requested_state_id = state.state_id  # set start after switch

    def get_current_states(self):
        """Get all currently active (sub-)states."""
        return self._state_machine.get_deep_states()

    def get_locked_state(self):
        """Return the first state designated as locked."""
        states = self._state_machine.get_deep_states()
        for state in states:
            while state is not None:
                if state.is_locked():
                    return state

                state = state._parent
        return None

    @classmethod
    def preempt(cls):
        """Preempt behavior."""
        PreemptableState.preempt = True

    # For internal use only

    def _get_state_machine(self):
        if self._state_machine is None:
            self._state_machine = self.create()  # pylint: disable=E1111
        return self._state_machine

    def _collect_contained(self, obj, path):
        contain_list = {path + '/' + key: value for (key, value) in getattr(obj, 'contains', {}).items()}
        add_to_list = {}
        for b_id, b_inst in contain_list.items():
            add_to_list.update(self._collect_contained(b_inst, b_id))
        contain_list.update(add_to_list)
        return contain_list

    def get_contained_behaviors(self):
        """Get contained behaviors within this behavior."""
        return self._collect_contained(self, '')

    def _set_typed_attribute(self, name, value):
        attr = getattr(self, name)
        # convert type if required
        if not isinstance(value, type(attr)):
            if isinstance(attr, int):
                value = int(value)
            elif isinstance(attr, float):
                value = float(value)
            elif isinstance(attr, bool):
                value = (value != '0' and value.lower() != 'false')
            elif isinstance(attr, dict):
                import yaml  # pylint: disable=C0415
                value = getattr(yaml, 'unsafe_load', yaml.load)(value)
        setattr(self, name, value)

    def set_up(self, beh_id, autonomy_level, debug):
        """Set up the behavior."""
        self.beh_id = beh_id
        self._autonomy_level = autonomy_level
        self._debug = debug

    def _get_states_of_path(self, path, container):
        path_elements = path.split('/')
        if len(path_elements) < 2:
            return [container]  # actually a state in this case
        state_label = path_elements[1]
        new_path = '/'.join(path_elements[1:])
        # collect along the path and append self
        if state_label in container:
            childlist = self._get_states_of_path(new_path, container[state_label])
            if childlist is None:
                return None
            childlist.append(container)
            return childlist

        return None
