#!/usr/bin/env python3

"""Synthetic behavior used by the standalone onboard profiling script."""

from flexbe_core import Autonomy, Behavior, EventState, OperatableStateMachine, initialize_flexbe_core


class _ProfileStepState(EventState):
    """Tiny state whose cost is dominated by the behavior engine around it."""

    def __init__(self, payload, ticks_per_state, desired_rate_hz):
        super().__init__(
            outcomes=['done'],
            input_keys=['profile_accumulator'],
            output_keys=['profile_accumulator'],
            desired_rate=desired_rate_hz
        )
        self._payload = payload
        self._ticks_per_state = max(1, int(ticks_per_state))
        self._ticks_remaining = self._ticks_per_state

    def on_enter(self, userdata):
        self._ticks_remaining = self._ticks_per_state

    def execute(self, userdata):
        total = userdata.profile_accumulator or 0
        for value in range(self._payload):
            total += value
        userdata.profile_accumulator = total
        self._ticks_remaining -= 1
        if self._ticks_remaining <= 0:
            return 'done'
        return None


class OnboardProfileBehaviorSM(Behavior):
    """Parameterized behavior that creates a large nested state-machine graph."""

    __test__ = False

    def __init__(self, node):
        super().__init__()
        self.name = 'Onboard Profile Behavior'
        self.node = node
        initialize_flexbe_core(node)

        self.add_parameter('depth', 6)
        self.add_parameter('width', 8)
        self.add_parameter('branches', 4)
        self.add_parameter('payload', 0)
        self.add_parameter('ticks_per_state', 5)
        self.add_parameter('state_rate_hz', 200.0)

    def create(self):
        """Create the profiling state machine."""
        depth = max(1, int(self.depth))
        width = max(1, int(self.width))
        branches = max(1, int(self.branches))
        payload = max(0, int(self.payload))
        ticks_per_state = max(1, int(self.ticks_per_state))
        state_rate_hz = max(1.0, float(self.state_rate_hz))

        state_machine = OperatableStateMachine(
            outcomes=['finished', 'failed'],
            output_keys=['profile_accumulator']
        )
        state_machine.userdata.profile_accumulator = 0

        branch_machines = [
            self._build_branch(depth, width, payload, ticks_per_state, state_rate_hz, branch_index, level=0)
            for branch_index in range(branches)
        ]

        with state_machine:
            for branch_index, branch_machine in enumerate(branch_machines):
                label = f'branch_{branch_index}'
                next_outcome = f'branch_{branch_index + 1}' if branch_index + 1 < branches else 'finished'
                OperatableStateMachine.add(
                    label,
                    branch_machine,
                    transitions={'finished': next_outcome},
                    autonomy={'finished': Autonomy.Off}
                )

        return state_machine

    def _build_branch(self, depth, width, payload, ticks_per_state, state_rate_hz, branch_index, level):
        """Recursively build a nested branch."""
        state_machine = OperatableStateMachine(
            outcomes=['finished'],
            input_keys=['profile_accumulator'],
            output_keys=['profile_accumulator']
        )

        with state_machine:
            for step_index in range(width):
                label = f'level_{level}_step_{step_index}'
                if step_index + 1 < width:
                    next_label = f'level_{level}_step_{step_index + 1}'
                elif level + 1 < depth:
                    next_label = f'level_{level}_sub_sm'
                else:
                    next_label = 'finished'

                OperatableStateMachine.add(
                    label,
                    _ProfileStepState(
                        payload + branch_index + level + step_index,
                        ticks_per_state,
                        state_rate_hz
                    ),
                    transitions={'done': next_label},
                    autonomy={'done': Autonomy.Off},
                    remapping={'profile_accumulator': 'profile_accumulator'}
                )

            if level + 1 < depth:
                child = self._build_branch(
                    depth, width, payload, ticks_per_state, state_rate_hz, branch_index, level + 1
                )
                OperatableStateMachine.add(
                    f'level_{level}_sub_sm',
                    child,
                    transitions={'finished': 'finished'},
                    autonomy={'finished': Autonomy.Off},
                    remapping={'profile_accumulator': 'profile_accumulator'}
                )

        return state_machine
