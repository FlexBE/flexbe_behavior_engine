#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
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
#    * Neither the name of the Christopher Newport University nor the names of its
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

"""Unit tests for behavior and priority guard paths."""
import threading
import unittest
from unittest.mock import patch

from flexbe_core.behavior import Behavior
from flexbe_core.core import (
    ConcurrencyContainer,
    OperatableStateMachine,
    PreemptableState,
    PreemptableStateMachine,
    RosState,
    RosStateMachine,
    State,
    StateMachine,
    StateMachineError,
    StateMap,
)
from flexbe_core.core.event_state import EventState
from flexbe_core.core.lockable_state_machine import LockableStateMachine
from flexbe_core.core.priority_container import PriorityContainer
from flexbe_core.core.topics import Topics
from flexbe_core.core.user_data import UserData


class _FakeStateMap:

    def __getitem__(self, _key):
        return None


class _FakeStateMachine:

    def confirm(self, *_args, **_kwargs):
        return None


class _DestroyTrackingStateMachine:

    def __init__(self, spin_exc=None):
        self.spin_exc = spin_exc
        self.destroy_called = False

    def spin(self):
        if self.spin_exc is not None:
            raise self.spin_exc
        return 'finished'

    def destroy(self):
        self.destroy_called = True


class _FakeParent:

    def __init__(self, path):
        self.path = path

    @property
    def path_segments(self):
        return tuple(p for p in self.path.split('/') if p)


class _FakeGlobalPreemptParent(_FakeParent):

    def __init__(self, path=''):
        super().__init__(path)
        self._handles_preempt_globally = True
        self._global_preempt_subscription_active = True


class _DoneState(State):

    def __init__(self):
        super().__init__(outcomes=['done'])

    def execute(self, userdata):
        return 'done'


class _PassiveState(State):

    def __init__(self, name):
        super().__init__(outcomes=['done'])
        self._name = name


class _UnnamedPassiveState(State):

    def __init__(self):
        super().__init__(outcomes=['done'])


class _TrackingLockableStateMachine(LockableStateMachine):

    def __init__(self):
        super().__init__(outcomes=['done'])
        self.super_execute_calls = 0

    def _notify_start(self):
        return None

    def _notify_stop(self):
        return None

    def wait(self, target_wakeup_ns=None):
        return None

    def _execute_current_state(self):
        return None

    def execute(self, userdata):
        return super().execute(userdata)


class _FalsyOutcomeState(State):

    def __init__(self):
        super().__init__(outcomes=[''])

    def execute(self, userdata):
        return ''


class _TrackingStateMachine(StateMachine):

    def __init__(self, outcomes=None):
        super().__init__(outcomes=outcomes if outcomes is not None else [''])
        self.published_outcomes = []
        self.on_exit_calls = 0

    def wait(self, target_wakeup_ns=None):
        return None

    def on_exit(self, userdata):
        self.on_exit_calls += 1
        self._exited = True
        self._entering = True
        self._current_state = None

    def _publish_outcome(self, outcome):
        self.published_outcomes.append(outcome)


_DoneTrackingStateMachine = lambda: _TrackingStateMachine(outcomes=['done'])  # noqa: E731


class _WrapperTrackingState(State):

    def __init__(self):
        super().__init__(outcomes=['done'], input_keys=['value'], output_keys=['value'])
        self.wrapper_ids = []
        self.execute_calls = 0

    def execute(self, userdata):
        self.wrapper_ids.append(id(userdata))
        userdata.value = (userdata.value or 0) + 1
        self.execute_calls += 1
        if self.execute_calls >= 2:
            return 'done'
        return None


class _FakeClockTime:

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _FakeClock:

    def __init__(self, nanoseconds):
        self._nanoseconds = nanoseconds

    def now(self):
        return _FakeClockTime(self._nanoseconds)


class _FakeNode:

    def __init__(self, nanoseconds):
        self._clock = _FakeClock(nanoseconds)

    def get_clock(self):
        return self._clock


class _FakeProxyFactory:

    def __call__(self, *args, **kwargs):
        return self


class _ChildBehavior:

    def __init__(self):
        self.threshold = 0
        self.contains = {}

    def _set_typed_attribute(self, name, value):
        setattr(self, name, value)


class _ChildBehaviorWithStateMachine(_ChildBehavior):

    def __init__(self, state_machine):
        super().__init__()
        self._state_machine = state_machine

    def _get_state_machine(self):
        return self._state_machine


class _FakeProxyPublisher:

    created_topics = []
    removed_topics = []

    @classmethod
    def reset(cls):
        cls.created_topics = []
        cls.removed_topics = []

    def create_publisher(self, topic, msg_type, qos=None):  # pylint: disable=unused-argument
        self.created_topics.append(topic)

    def remove_publisher(self, topic):
        self.removed_topics.append(topic)

    def publish(self, topic, msg):
        return None


class _FakeProxySubscriber:

    subscribed = []
    unsubscribed = []
    removed_last = []

    @classmethod
    def reset(cls):
        cls.subscribed = []
        cls.unsubscribed = []
        cls.removed_last = []

    def subscribe(self, topic, msg_type, callback=None, inst_id=None, callback_group=None, qos=None):  # noqa: ARG002
        self.subscribed.append((topic, inst_id))
        return None

    def unsubscribe_topic(self, topic, inst_id=None):  # noqa: ARG002
        self.unsubscribed.append((topic, inst_id))
        return None

    def enable_buffer(self, topic):
        return None

    def has_msg(self, topic):
        return False

    def get_last_msg(self, topic):
        raise KeyError(topic)

    def remove_last_msg(self, topic, clear_buffer=False):  # noqa: ARG002
        self.removed_last.append(topic)
        return None


class TestBehaviorAndPriorityGuards(unittest.TestCase):
    """Validate defensive behavior for state-map and priority-container error paths."""

    def test_set_typed_attribute_preserves_boolean_type(self):
        """Behavior boolean parameters should parse string inputs as bools, not ints."""
        behavior = object.__new__(Behavior)
        behavior.enabled = True

        Behavior._set_typed_attribute(behavior, 'enabled', 'false')
        self.assertIs(behavior.enabled, False)
        self.assertIsInstance(behavior.enabled, bool)

        Behavior._set_typed_attribute(behavior, 'enabled', '1')
        self.assertIs(behavior.enabled, True)
        self.assertIsInstance(behavior.enabled, bool)

    def test_set_typed_attribute_parses_plain_yaml_dicts_safely(self):
        """Behavior dict parameters should accept plain YAML mappings via the safe loader."""
        behavior = object.__new__(Behavior)
        behavior.config = {'existing': True}

        Behavior._set_typed_attribute(behavior, 'config', 'speed: fast\ncount: 2\n')

        self.assertEqual({'speed': 'fast', 'count': 2}, behavior.config)

    def test_set_typed_attribute_rejects_unsafe_yaml_tags_for_dicts(self):
        """Behavior dict parameters should reject Python-specific YAML constructors."""
        behavior = object.__new__(Behavior)
        behavior.config = {}

        with self.assertRaises(Exception):
            Behavior._set_typed_attribute(behavior, 'config', '!!python/object/apply:os.system ["echo blocked"]')

    def test_set_parameter_ignores_sibling_prefix_behavior_ids(self):
        """Behavior.set_parameter should not update sibling contained behaviors with matching prefixes."""
        behavior = object.__new__(Behavior)
        foo = _ChildBehavior()
        foo_bar = _ChildBehavior()
        behavior.get_contained_behaviors = lambda: {'/foo': foo, '/foo_bar': foo_bar}

        found = Behavior.set_parameter(behavior, '/foo/threshold', 7)

        self.assertTrue(found)
        self.assertEqual(foo.threshold, 7)
        self.assertEqual(foo_bar.threshold, 0)

    def test_use_behavior_applies_parameters_and_prunes_non_default_userdata(self):
        """Behavior.use_behavior should set child parameters and drop runtime-overridden userdata keys."""
        behavior = Behavior()

        class _StateMachineStub:

            def __init__(self):
                self._input_keys = ['keep_default', 'override_at_runtime']
                self._own_userdata = UserData()
                self._own_userdata._data = {
                    'keep_default': 'default-value',
                    'override_at_runtime': 'runtime-value',
                }

        state_machine = _StateMachineStub()
        child = _ChildBehaviorWithStateMachine(state_machine)
        behavior.contains['child'] = child

        used_state_machine = behavior.use_behavior(
            _ChildBehaviorWithStateMachine,
            'child',
            default_keys=['keep_default'],
            parameters={'threshold': 9},
        )

        self.assertIs(used_state_machine, state_machine)
        self.assertEqual(child.threshold, 9)
        self.assertEqual(state_machine._input_keys, ['override_at_runtime'])
        self.assertIn('keep_default', state_machine._own_userdata._data)
        self.assertNotIn('override_at_runtime', state_machine._own_userdata._data)

    def test_prepare_for_execution_sets_autonomy_and_input_userdata(self):
        """Behavior.prepare_for_execution should build the machine and seed matching userdata inputs."""
        original_autonomy = OperatableStateMachine.autonomy_level
        behavior = Behavior()
        behavior._autonomy_level = 1

        class _StateMachineStub:

            def __init__(self):
                self._own_userdata = {'accepted': 'old', 'untouched': 'keep'}

        state_machine = _StateMachineStub()
        behavior.create = lambda: state_machine

        try:
            behavior.prepare_for_execution({'accepted': 7, 'ignored': 9})
            configured_autonomy = OperatableStateMachine.autonomy_level
        finally:
            OperatableStateMachine.autonomy_level = original_autonomy

        self.assertIs(behavior._state_machine, state_machine)
        self.assertEqual(configured_autonomy, 1)
        self.assertEqual(state_machine._own_userdata['accepted'], 7)
        self.assertEqual(state_machine._own_userdata['untouched'], 'keep')
        self.assertNotIn('ignored', state_machine._own_userdata)

    def test_behavior_setup_add_parameter_and_lazy_state_machine_creation(self):
        """Behavior helpers should expose parameter/setup state and only build the machine once."""
        behavior = Behavior()
        created = []
        state_machine = object()

        behavior.add_parameter('threshold', 5)
        behavior.set_up(99, 2, True)
        behavior.create = lambda: created.append('created') or state_machine

        first = behavior._get_state_machine()
        second = behavior._get_state_machine()

        self.assertEqual(behavior.threshold, 5)
        self.assertEqual(behavior.beh_id, 99)
        self.assertEqual(behavior._autonomy_level, 2)
        self.assertTrue(behavior._debug)
        self.assertIs(first, state_machine)
        self.assertIs(second, state_machine)
        self.assertEqual(created, ['created'])

    def test_add_behavior_registers_child_instance(self):
        """Behavior.add_behavior should instantiate and store the contained behavior by id."""
        behavior = Behavior()
        sentinel_node = object()

        class _NestedBehavior:

            def __init__(self, node):
                self.node = node

        behavior.add_behavior(_NestedBehavior, 'nested', sentinel_node)

        self.assertIn('nested', behavior.contains)
        self.assertIsInstance(behavior.contains['nested'], _NestedBehavior)
        self.assertIs(behavior.contains['nested'].node, sentinel_node)

    def test_use_behavior_returns_none_for_missing_child(self):
        """Behavior.use_behavior should fail softly when the contained behavior was never added."""
        behavior = Behavior()

        with patch('flexbe_core.behavior.Logger.logerr') as logerr:
            result = behavior.use_behavior(_ChildBehaviorWithStateMachine, 'missing')

        self.assertIsNone(result)
        logerr.assert_called_once()

    def test_get_state_by_id_uses_state_map_get_state(self):
        """Behavior.get_state_by_id should delegate to StateMap.get_state."""
        behavior = object.__new__(Behavior)
        sentinel = object()

        class _Map:

            def get_state(self, _st_id):
                return sentinel

        behavior._state_map = _Map()
        self.assertIs(behavior.get_state_by_id(1234), sentinel)

    def test_behavior_getters_delegate_when_state_machine_exists(self):
        """Behavior helper getters should delegate to the active state machine when present."""
        behavior = object.__new__(Behavior)
        latest_status = object()
        deep_states = ('root', 'leaf')

        class _StateMachine:

            @staticmethod
            def get_latest_status():
                return latest_status

            @staticmethod
            def get_deep_states():
                return deep_states

        class _StateMap:
            items = [(1, '/root')]

        behavior._state_machine = _StateMachine()
        behavior._state_map = _StateMap()

        self.assertIs(behavior.get_latest_status(), latest_status)
        self.assertEqual(behavior.get_current_states(), deep_states)
        self.assertEqual(behavior.state_map_items, [(1,), ('/root',)])

    def test_behavior_getters_fall_back_when_state_machine_is_absent(self):
        """Behavior helper getters should return empty defaults before a machine exists."""
        behavior = object.__new__(Behavior)
        behavior._state_machine = None
        behavior._state_map = None

        status = behavior.get_latest_status()

        self.assertEqual(status.behavior_id, 0)
        self.assertEqual(list(status.current_state_checksums), [])
        self.assertEqual(behavior.state_map_items, ([], []))
        self.assertIsNone(behavior.get_state_by_id(99))

    def test_behavior_collects_nested_containments_and_finds_locked_state(self):
        """Behavior helper traversal should recurse through contained behaviors and active ancestors."""
        behavior = Behavior()
        child = _ChildBehavior()
        grandchild = _ChildBehavior()
        child.contains['grand'] = grandchild
        behavior.contains['child'] = child

        locked = type('_Locked', (), {'is_locked': staticmethod(lambda: True), '_parent': None})()
        leaf = type('_Leaf', (), {'is_locked': staticmethod(lambda: False), '_parent': locked})()
        behavior._state_machine = type('_SM', (), {'get_deep_states': staticmethod(lambda: (leaf,))})()

        contained = behavior.get_contained_behaviors()

        self.assertEqual(contained['/child'], child)
        self.assertEqual(contained['/child/grand'], grandchild)
        self.assertIs(behavior.get_locked_state(), locked)

    def test_prepare_for_switch_rejects_container_anchor(self):
        """Behavior switches should reject state-machine containers as anchors."""
        behavior = object.__new__(Behavior)
        container = object.__new__(StateMachine)

        with self.assertRaises(ValueError):
            behavior.prepare_for_switch(container)

    def test_prepare_for_switch_rehomes_matching_state_path(self):
        """Behavior switches should rehome the active state into the matching container path."""
        behavior = object.__new__(Behavior)

        class _SwitchContainer:

            def __init__(self, name, userdata, parent=None):
                self.name = name
                self.userdata = userdata
                self._parent = parent
                self.children = {}
                self.replaced_userdata = []
                self.replaced_state = None
                self.removed_state = None

            def __contains__(self, label):
                return label in self.children

            def __getitem__(self, label):
                return self.children[label]

            def replace_userdata(self, userdata):
                self.replaced_userdata.append(userdata)

            def replace_state(self, state):
                self.replaced_state = state

            def remove_state(self, state):
                self.removed_state = state

        new_top = _SwitchContainer('top', userdata='new-top')
        new_container = _SwitchContainer('container', userdata='new-container', parent=new_top)
        new_leaf_placeholder = object()
        new_top.children['container'] = new_container
        new_container.children['leaf'] = new_leaf_placeholder
        behavior._state_machine = new_top

        old_top = _SwitchContainer('old-top', userdata='old-top')
        old_container = _SwitchContainer('old-container', userdata='old-container', parent=old_top)
        active_state = type(
            '_State',
            (),
            {
                'name': 'leaf',
                'path': '/container/leaf',
                'state_id': 77,
                '_parent': old_container,
                '_locked': False,
            },
        )()

        behavior.prepare_for_switch(active_state)

        self.assertTrue(active_state._locked)
        self.assertIs(old_container.removed_state, active_state)
        self.assertEqual(new_container.replaced_userdata, ['old-container'])
        self.assertEqual(new_top.replaced_userdata, ['old-top'])
        self.assertIs(new_container.replaced_state, active_state)
        self.assertEqual(behavior.requested_state_id, 77)

    def test_get_states_of_path_handles_root_and_missing_segments(self):
        """Behavior path lookup should return the root container or None for unknown descendants."""
        behavior = object.__new__(Behavior)

        class _Container:

            def __contains__(self, label):
                return False

        container = _Container()

        self.assertEqual(behavior._get_states_of_path('root', container), [container])
        self.assertIsNone(behavior._get_states_of_path('/root/child', container))

    def test_execute_destroys_state_machine_when_spin_raises(self):
        """Behavior.execute should always destroy the state machine on execution failure."""
        behavior = object.__new__(Behavior)
        state_machine = _DestroyTrackingStateMachine(spin_exc=RuntimeError('boom'))
        behavior._state_machine = state_machine

        with self.assertRaises(RuntimeError):
            behavior.execute()

        self.assertTrue(state_machine.destroy_called)
        self.assertIsNone(behavior._state_machine)

    def test_confirm_sets_switch_path_for_requested_state(self):
        """Behavior.confirm should prime the switch path when the requested state exists."""
        behavior = object.__new__(Behavior)
        confirm_calls = []

        class _StateMachine:

            def confirm(self, name, beh_id, state_map):
                confirm_calls.append((name, beh_id, state_map))

        class _RequestedStateMap:

            def __getitem__(self, key):
                if key == 123:
                    return type('_RequestedState', (), {'path': '/root/child'})()
                return None

        behavior._state_machine = _StateMachine()
        behavior.name = 'test'
        behavior.beh_id = 7
        behavior.requested_state_id = 123

        with patch('flexbe_core.behavior.StateMap', _RequestedStateMap), \
                patch('flexbe_core.behavior.LockableStateMachine.clear_path_for_switch') as clear_path, \
                patch('flexbe_core.behavior.LockableStateMachine.set_path_for_switch') as set_path:
            behavior.confirm()

        self.assertEqual(len(confirm_calls), 1)
        self.assertEqual(confirm_calls[0][:2], ('test', 7))
        clear_path.assert_called_once_with()
        set_path.assert_called_once_with('/root/child')

    def test_confirm_raises_for_invalid_requested_state_id(self):
        """Behavior.confirm should raise controlled error for unknown requested state id."""
        behavior = object.__new__(Behavior)
        behavior._state_machine = _FakeStateMachine()
        behavior.name = 'test'
        behavior.beh_id = 7
        behavior.requested_state_id = 123

        with patch('flexbe_core.behavior.StateMap', _FakeStateMap):
            with self.assertRaises(RuntimeError):
                behavior.confirm()

    def test_priority_execute_restores_active_container_on_exception(self):
        """PriorityContainer.execute should reset active_container when execute raises."""
        container = object.__new__(PriorityContainer)
        container._name = 'priority'
        container._parent = _FakeParent('/root')
        container._parent_active_container = None

        PriorityContainer.active_container = None
        with patch('flexbe_core.core.priority_container.OperatableStateMachine.execute',
                   side_effect=RuntimeError('boom')):
            with self.assertRaises(RuntimeError):
                container.execute(None)

        self.assertIsNone(PriorityContainer.active_container)

    def test_confirm_raises_when_structure_build_fails(self):
        """OperatableStateMachine.confirm should fail fast on invalid structure metadata."""
        sm = OperatableStateMachine(outcomes=['finished'])
        with sm:
            OperatableStateMachine.add('state',
                                       _DoneState(),
                                       transitions={'done': 'finished'},
                                       autonomy={})

        with patch('flexbe_core.core.operatable_state_machine.Logger.logerr'), \
                patch('flexbe_core.core.operatable_state_machine.Logger.localerr'):
            with self.assertRaises(StateMachineError) as ctx:
                sm.confirm('test_sm', 1, StateMap())

        self.assertIn("Failed to build container structure for 'state'", str(ctx.exception))

    def test_concurrency_get_deep_states_only_reports_active_children(self):
        """Reports only active children in ConcurrencyContainer status."""
        cc = object.__new__(ConcurrencyContainer)
        cc._name = 'cc'
        cc._deep_states_list_cache = None
        cc._deep_states_cache_key = None
        cc._deep_states_cache_active_states = None
        cc._current_state = []
        active = _PassiveState('active')
        finished = _PassiveState('finished')
        cc._states = [active, finished]
        cc._current_state = [active]

        deep_states = cc.get_deep_states()

        self.assertEqual(deep_states, (cc, active))

    def test_switch_path_exact_container_match_does_not_crash(self):
        """Exact container switch targets should clear the marker without indexing into empty path segments."""
        sm = _TrackingLockableStateMachine()
        sm._name = 'foo'
        sm._parent = _FakeParent('')
        sm._labels = {'child': object()}
        LockableStateMachine.set_path_for_switch('/foo')

        try:
            with patch('flexbe_core.core.lockable_state_machine.RosStateMachine.execute', return_value=None):
                sm.execute(None)
        finally:
            LockableStateMachine.clear_path_for_switch()

        self.assertIsNone(sm._current_state)

    def test_switch_path_sibling_prefix_is_ignored(self):
        """Sibling-prefixed switch paths should not be treated as descendant matches."""
        sm = _TrackingLockableStateMachine()
        sm._name = 'foo'
        sm._parent = _FakeParent('')
        child = object()
        sm._labels = {'child': child}
        LockableStateMachine.set_path_for_switch('/foo_bar/child')

        try:
            with patch('flexbe_core.core.lockable_state_machine.RosStateMachine.execute', return_value=None):
                sm.execute(None)
        finally:
            LockableStateMachine.clear_path_for_switch()

        self.assertIsNone(sm._current_state)

    def test_lockable_state_machine_transition_allowed_honors_internal_and_parent_rules(self):
        """Lockable transitions should allow internal targets while deferring external ones to lock state and parent."""
        sm = object.__new__(LockableStateMachine)
        sm._labels = {'inside': object()}
        sm._transitions = {
            'state': {
                'internal': 'inside',
                'external': 'outside',
            }
        }

        class _Parent:

            def __init__(self, allowed):
                self.allowed = allowed
                self.calls = []

            def transition_allowed(self, state, transition_target):
                self.calls.append((state, transition_target))
                return self.allowed

        denied_parent = _Parent(False)
        allowed_parent = _Parent(True)

        sm._locked = True
        sm._parent = denied_parent
        sm._name = 'container'
        self.assertTrue(sm.transition_allowed('state', None))
        self.assertTrue(sm.transition_allowed('state', 'internal'))
        self.assertFalse(sm.transition_allowed('state', 'external'))
        self.assertEqual(denied_parent.calls, [])

        sm._locked = False
        self.assertFalse(sm.transition_allowed('state', 'external'))
        self.assertEqual(denied_parent.calls, [('container', 'outside')])

        sm._parent = allowed_parent
        self.assertTrue(sm.transition_allowed('state', 'external'))
        self.assertEqual(allowed_parent.calls, [('container', 'outside')])

    def test_lockable_state_machine_reports_recursive_locked_child(self):
        """Recursive lock helpers should find locked children inside nested lockable containers."""
        root = object.__new__(LockableStateMachine)
        nested = object.__new__(LockableStateMachine)

        class _Leaf:

            def __init__(self):
                self._locked = True

            def is_locked(self):
                return self._locked

        leaf = _Leaf()

        root._locked = False
        nested._locked = False

        root._states = [nested]
        nested._states = [leaf]

        self.assertTrue(root.is_locked_inside())
        self.assertIs(root.get_locked_state(), leaf)
        self.assertFalse(root.is_locked())
        self.assertTrue(leaf.is_locked())

    def test_lockable_state_machine_replaces_userdata_and_manages_child_state_entries(self):
        """Lockable helpers should replace userdata, swap child states, and remove labels consistently."""
        sm = object.__new__(LockableStateMachine)
        old_parent = object()
        old_state = type('_State', (), {'name': 'child', '_parent': old_parent})()
        new_state = type('_State', (), {'name': 'child', '_parent': None})()
        replacement_userdata = {'value': 7}

        sm._userdata = None
        sm._labels = {'child': old_state}
        sm._states = [old_state]

        sm.replace_userdata(replacement_userdata)
        sm.replace_state(new_state)

        self.assertIs(sm._userdata, replacement_userdata)
        self.assertIs(sm._labels['child'], new_state)
        self.assertEqual(sm._states, [new_state])
        self.assertIs(new_state._parent, old_parent)

        sm.remove_state(new_state)
        self.assertEqual(sm._labels, {})
        self.assertEqual(sm._states, [])

    def test_lockable_state_machine_lock_and_unlock_delegate_to_parent_when_needed(self):
        """Lock and unlock should toggle locally, delegate upward, and return False at the root when unresolved."""
        delegated = []
        parent = type(
            '_Parent',
            (),
            {
                'lock': staticmethod(lambda state_id: delegated.append(('lock', state_id)) or True),
                'unlock': staticmethod(lambda state_id: delegated.append(('unlock', state_id)) or True),
            },
        )()

        sm = object.__new__(LockableStateMachine)
        sm._state_id = 42
        sm._locked = False
        sm._parent = parent

        self.assertTrue(sm.lock(42))
        self.assertTrue(sm.is_locked())
        self.assertTrue(sm.unlock(42))
        self.assertFalse(sm.is_locked())
        self.assertTrue(sm.lock(99))
        self.assertTrue(sm.unlock(99))
        self.assertEqual(delegated, [('lock', 99), ('unlock', 99)])

        root = object.__new__(LockableStateMachine)
        root._state_id = 77
        root._locked = False
        root._parent = None
        self.assertFalse(root.lock(99))
        self.assertFalse(root.unlock(99))

    def test_lockable_state_machine_internal_transition_and_direct_lock_queries(self):
        """Internal transition checks and direct lock queries should use local state cleanly."""
        sm = object.__new__(LockableStateMachine)
        sm._labels = {'inside': object()}
        sm._locked = True
        sm._states = []

        self.assertTrue(sm._is_internal_transition('inside'))
        self.assertFalse(sm._is_internal_transition('outside'))
        self.assertTrue(sm.is_locked_inside())
        self.assertIs(sm.get_locked_state(), sm)

    def test_priority_container_caches_active_container_segments(self):
        """Caches normalized active-container segments."""
        PriorityContainer.set_active_container('/alpha/beta')
        try:
            self.assertEqual(PriorityContainer.active_container, '/alpha/beta')
            self.assertEqual(PriorityContainer.active_container_segments, ('alpha', 'beta'))
        finally:
            PriorityContainer.set_active_container(None)

    def test_set_path_for_switch_caches_normalized_segments(self):
        """Caches normalized switch-path segments."""
        LockableStateMachine.set_path_for_switch('/foo/bar')
        try:
            self.assertEqual(LockableStateMachine.path_for_switch, '/foo/bar')
            self.assertEqual(LockableStateMachine.path_for_switch_segments, ('foo', 'bar'))
        finally:
            LockableStateMachine.clear_path_for_switch()

    def test_state_machine_treats_falsy_outcome_as_terminal(self):
        """Base StateMachine should exit on any non-None outcome, including falsy values."""
        sm = _TrackingStateMachine()
        with sm:
            StateMachine.add('state', _FalsyOutcomeState(), transitions={'': ''})

        outcome = sm.execute(None)

        self.assertEqual(outcome, '')
        self.assertEqual(sm.on_exit_calls, 1)
        self.assertTrue(sm._exited)
        self.assertEqual(sm.published_outcomes, [''])

    def test_get_deep_states_reuses_cache_until_nested_state_changes(self):
        """Reuses cached deep-state traversals until a child invalidates them."""
        root = StateMachine(outcomes=['done'])
        child = StateMachine(outcomes=['done'])
        first = _UnnamedPassiveState()
        second = _UnnamedPassiveState()

        with child:
            StateMachine.add('first', first, transitions={'done': 'done'})
            StateMachine.add('second', second, transitions={'done': 'done'})
        with root:
            StateMachine.add('child', child, transitions={'done': 'done'})

        root._current_state = child
        child._current_state = first

        deep_states = root.get_deep_states()
        cached_deep_states = root.get_deep_states()
        self.assertIs(deep_states, cached_deep_states)
        self.assertEqual(deep_states, (root, child, first))

        child._current_state = second
        child._invalidate_deep_states_cache()

        updated_deep_states = root.get_deep_states()
        self.assertIsNot(updated_deep_states, deep_states)
        self.assertEqual(updated_deep_states, (root, child, second))

    def test_concurrency_get_deep_states_recomputes_when_active_children_change(self):
        """Refreshes cached deep states when concurrency membership changes."""
        cc = object.__new__(ConcurrencyContainer)
        cc._name = 'cc'
        cc._parent = None
        cc._deep_states_list_cache = None
        cc._deep_states_cache_key = None
        cc._deep_states_cache_active_states = None

        first = _PassiveState('first')
        second = _PassiveState('second')
        cc._current_state = [first]

        deep_states = cc.get_deep_states()
        cached_deep_states = cc.get_deep_states()
        self.assertIs(deep_states, cached_deep_states)
        self.assertEqual(deep_states, (cc, first))

        cc._current_state = [first, second]

        updated_deep_states = cc.get_deep_states()
        self.assertIsNot(updated_deep_states, deep_states)
        self.assertEqual(updated_deep_states, (cc, first, second))

    def test_get_latest_status_reuses_cached_message_until_active_states_change(self):
        """Reuses the cached BehaviorSync message while active states are unchanged."""
        sm = object.__new__(OperatableStateMachine)
        sm._status_lock = threading.Lock()
        sm._last_deep_states_list = []
        sm._latest_status_msg_cache = None
        sm._latest_status_active_states = None
        sm._latest_status_behavior_id = None
        sm.id = 42

        status = OperatableStateMachine.get_latest_status(sm)
        cached_status = OperatableStateMachine.get_latest_status(sm)
        self.assertIs(status, cached_status)
        self.assertEqual(status.behavior_id, 42)
        self.assertEqual(list(status.current_state_checksums), [])

        active = _PassiveState('active')
        active._outcomes = ['done']
        active._last_outcome = None
        sm._last_deep_states_list = [active]

        updated_status = OperatableStateMachine.get_latest_status(sm)
        self.assertIsNot(updated_status, status)

    def test_process_sync_request_reissues_only_deepest_requested_outcome(self):
        """Explicit sync should resend only the deepest pending outcome request."""
        sm = object.__new__(OperatableStateMachine)
        published = []
        sync_status = object()

        class _Pub:

            def publish(self, topic, msg):
                published.append((topic, msg))

        shallow = type(
            '_State',
            (),
            {
                '_last_requested_outcome': 'fallback',
                'outcomes': ['fallback'],
                'state_id': 10,
                'path': '/root',
            },
        )()
        deep = type(
            '_State',
            (),
            {
                '_last_requested_outcome': 'done',
                'outcomes': ['done', 'failed'],
                'state_id': 20,
                'path': '/root/child',
            },
        )()

        sm._inner_sync_request = True
        sm._pub = _Pub()
        sm._last_deep_states_list = [shallow, deep]
        sm.get_latest_status = lambda: sync_status

        with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'), \
                patch('flexbe_core.core.operatable_state_machine.Logger.localerr'):
            OperatableStateMachine.process_sync_request(sm)

        self.assertFalse(sm._inner_sync_request)
        self.assertEqual(
            [topic for topic, _msg in published],
            [
                Topics._MIRROR_SYNC_TOPIC,
                Topics._CMD_FEEDBACK_TOPIC,
                Topics._OUTCOME_REQUEST_TOPIC,
            ],
        )
        self.assertIs(published[0][1], sync_status)
        self.assertEqual(published[1][1].command, 'sync')
        self.assertEqual(list(published[1][1].args), [])
        self.assertEqual(published[2][1].target, 20)
        self.assertEqual(published[2][1].outcome, 0)

    def test_operatable_state_machine_requests_operator_confirmation_once(self):
        """Controlled state machines should publish one outcome request when autonomy blocks a transition."""
        sm = object.__new__(OperatableStateMachine)
        published = []
        state_logger_calls = []

        class _Sub:

            @staticmethod
            def peek_if_buffered(_topic):
                return None

        class _Pub:

            def publish(self, topic, msg):
                published.append((topic, msg))

        class _Parent:
            autonomy_level = 0
            path = '/root'

            @staticmethod
            def is_transition_allowed(_label, _outcome):
                return False

            @staticmethod
            def get_required_autonomy(outcome, state):  # noqa: ARG004
                return 2 if outcome == 'done' else 0

        sm._is_controlled = True
        sm._state_id = 44
        sm._outcomes = ['done']
        sm._name = 'sm'
        sm._path = '/sm'
        sm._sub = _Sub()
        sm._pub = _Pub()
        sm._parent = _Parent()
        sm._force_transition = False
        sm._last_requested_outcome = None
        sm._manual_transition_requested = None
        sm._last_exception = None
        sm._last_outcome = None
        sm._breakpoint = False

        with patch('flexbe_core.core.operatable_state_machine.PreemptableStateMachine._execute_current_state',
                   return_value='done'), \
                patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'), \
                patch('flexbe_core.core.operatable_state_machine.Logger.loginfo_throttle'), \
                patch('flexbe_core.core.operatable_state_machine.StateLogger.log',
                      side_effect=lambda *args, **kwargs: state_logger_calls.append((args, kwargs))):
            first_outcome = OperatableStateMachine._execute_current_state(sm)
            second_outcome = OperatableStateMachine._execute_current_state(sm)

        self.assertIsNone(first_outcome)
        self.assertIsNone(second_outcome)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0][0], Topics._OUTCOME_REQUEST_TOPIC)
        self.assertEqual(published[0][1].target, 44)
        self.assertEqual(published[0][1].outcome, 0)
        self.assertEqual(sm._last_requested_outcome, 'done')
        self.assertEqual(sm._last_outcome, None)
        self.assertEqual(len(state_logger_calls), 1)
        self.assertEqual(state_logger_calls[0][1]['request'], 'done')
        self.assertEqual(state_logger_calls[0][1]['autonomy'], 0)
        self.assertEqual(state_logger_calls[0][1]['required'], 2)

    def test_operatable_state_machine_publish_outcome_marks_forced_transition(self):
        """Forced outcomes should publish the hash and record forced-transition metadata."""
        sm = object.__new__(OperatableStateMachine)
        published = []
        forced_logs = []

        class _Pub:

            def publish(self, topic, msg):
                published.append((topic, msg))

            @staticmethod
            def number_of_subscribers(_topic):
                return 0

        sm._pub = _Pub()
        sm._outcomes = ['done']
        sm._state_id = 55
        sm._name = 'sm'
        sm._path = '/sm'
        sm._force_transition = True
        sm._last_requested_outcome = 'done'

        with patch('flexbe_core.core.operatable_state_machine.StateLogger.log',
                   side_effect=lambda *args, **kwargs: forced_logs.append((args, kwargs))):
            OperatableStateMachine._publish_outcome(sm, 'done')

        self.assertEqual(len(published), 1)
        self.assertEqual(published[0][0], Topics._OUTCOME_TOPIC)
        self.assertEqual(published[0][1].data, StateMap.hash(sm, 0))
        self.assertEqual(len(forced_logs), 1)
        self.assertEqual(forced_logs[0][1]['type'], 'forced')
        self.assertEqual(forced_logs[0][1]['forced'], 'done')
        self.assertEqual(forced_logs[0][1]['requested'], 'done')
        self.assertIsNone(sm._last_requested_outcome)

    def test_operatable_state_machine_publish_outcome_handles_preempted(self):
        """Preempted outcomes should use the reserved max-outcome hash and always publish debug text."""
        sm = object.__new__(OperatableStateMachine)
        published = []

        class _Pub:

            def publish(self, topic, msg):
                published.append((topic, msg))

        sm._pub = _Pub()
        sm._state_id = 88
        sm._name = 'sm'
        sm._parent = type('_Parent', (), {'path': ''})()

        with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'):
            OperatableStateMachine._publish_outcome(sm, State._preempted_name)

        self.assertEqual(
            [topic for topic, _msg in published],
            [Topics._OUTCOME_TOPIC, Topics._DEBUG_TOPIC],
        )
        self.assertEqual(published[0][1].data, StateMap.hash(sm, StateMap._MAX_OUTCOME))
        self.assertEqual(published[1][1].data, '/sm > preempted')

    def test_operatable_state_machine_destroy_unsubscribes_and_removes_publishers(self):
        """Destroy should stop ROS control and release the top-level subscriptions and publishers."""
        sm = object.__new__(OperatableStateMachine)
        unsubscribed = []
        removed_publishers = []
        stopped = []
        disabled = []

        class _Sub:

            def unsubscribe_topic(self, topic, inst_id=None):
                unsubscribed.append((topic, inst_id))

        class _Pub:

            def remove_publisher(self, topic):
                removed_publishers.append(topic)

        sm._name = 'sm'
        sm.id = 123
        sm._sub = _Sub()
        sm._pub = _Pub()
        sm._notify_stop = lambda: stopped.append(True)
        sm._disable_ros_control = lambda: disabled.append(True)

        with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'), \
                patch('flexbe_core.core.operatable_state_machine.StateLogger.shutdown') as shutdown:
            OperatableStateMachine.destroy(sm)

        self.assertEqual([True], stopped)
        self.assertEqual([True], disabled)
        self.assertEqual(
            unsubscribed,
            [
                (Topics._CMD_ATTACH_TOPIC, id(sm)),
                (Topics._CMD_AUTONOMY_TOPIC, id(sm)),
                (Topics._CMD_SYNC_TOPIC, id(sm)),
                (Topics._REQUEST_STRUCTURE_TOPIC, id(sm)),
            ],
        )
        self.assertEqual(
            removed_publishers,
            [
                Topics._CMD_FEEDBACK_TOPIC,
                Topics._DEBUG_TOPIC,
                Topics._MIRROR_STRUCTURE_TOPIC,
                Topics._MIRROR_SYNC_TOPIC,
                Topics._OUTCOME_TOPIC,
                Topics._OUTCOME_REQUEST_TOPIC,
            ],
        )
        shutdown.assert_called_once_with()

    def test_operatable_state_machine_get_required_autonomy_logs_on_lookup_failure(self):
        """Autonomy lookup failures should be logged and return None instead of raising."""
        sm = object.__new__(OperatableStateMachine)
        sm._name = 'sm'
        sm._autonomy = {'other': {'done': 1}}
        sm._current_state = type('_State', (), {'name': 'other'})()
        mismatched_state = type('_State', (), {'name': 'leaf'})()

        with patch('flexbe_core.core.operatable_state_machine.Logger.error') as log_error, \
                patch('flexbe_core.core.operatable_state_machine.Logger.localerr') as local_error:
            result = OperatableStateMachine.get_required_autonomy(sm, 'done', mismatched_state)

        self.assertIsNone(result)
        log_error.assert_called_once()
        local_error.assert_called_once()

    def test_operatable_state_machine_set_autonomy_level_updates_and_ignores_invalid_values(self):
        """Autonomy updates should accept valid levels, reject invalid ones, and always acknowledge the command."""
        published = []
        sm = object.__new__(OperatableStateMachine)
        sm._name = 'sm'
        sm._pub = type('_Pub', (), {'publish': staticmethod(lambda topic, msg: published.append((topic, msg)))})()
        msg = type('_Msg', (), {'data': 2})()
        invalid_msg = type('_Msg', (), {'data': 9})()
        original_autonomy = OperatableStateMachine.autonomy_level

        try:
            OperatableStateMachine.autonomy_level = 1
            with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo') as localinfo, \
                    patch('flexbe_core.core.operatable_state_machine.Logger.logwarn') as logwarn:
                OperatableStateMachine._set_autonomy_level(sm, msg)
                self.assertEqual(OperatableStateMachine.autonomy_level, 2)
                localinfo.assert_called_once()

                OperatableStateMachine._set_autonomy_level(sm, invalid_msg)
                self.assertEqual(OperatableStateMachine.autonomy_level, 2)
                logwarn.assert_called_once()
        finally:
            OperatableStateMachine.autonomy_level = original_autonomy

        self.assertEqual([topic for topic, _ in published], [Topics._CMD_FEEDBACK_TOPIC, Topics._CMD_FEEDBACK_TOPIC])

    def test_operatable_state_machine_attach_callback_enables_control_and_requests_sync(self):
        """Attach requests should enable control, set autonomy, and publish command feedback."""
        published = []
        sm = object.__new__(OperatableStateMachine)
        sm._name = 'sm'
        sm._inner_sync_request = False
        sm._pub = type('_Pub', (), {'publish': staticmethod(lambda topic, msg: published.append((topic, msg)))})()
        enabled = []
        sm._enable_ros_control = lambda: enabled.append(True)
        original_autonomy = OperatableStateMachine.autonomy_level

        try:
            OperatableStateMachine.autonomy_level = 0
            msg = type('_Msg', (), {'data': 3})()
            with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'):
                OperatableStateMachine._attach_callback(sm, msg)
        finally:
            OperatableStateMachine.autonomy_level = original_autonomy

        self.assertEqual(enabled, [True])
        self.assertTrue(sm._inner_sync_request)
        self.assertEqual(published[0][0], Topics._CMD_FEEDBACK_TOPIC)
        self.assertEqual(published[0][1].command, 'attach')
        self.assertEqual(list(published[0][1].args), ['sm', '3'])

    def test_operatable_state_machine_mirror_structure_callback_handles_match_mismatch_and_missing_structure(self):
        """Mirror structure requests should publish only on matching ids and log otherwise."""
        published = []
        enabled = []
        sm = object.__new__(OperatableStateMachine)
        sm._name = 'sm'
        sm._inner_sync_request = False
        sm._pub = type('_Pub', (), {'publish': staticmethod(lambda topic, msg: published.append((topic, msg)))})()
        sm._enable_ros_control = lambda: enabled.append(True)
        sm._structure = type('_Structure', (), {'behavior_id': 7})()

        with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo') as localinfo, \
                patch('flexbe_core.core.operatable_state_machine.Logger.localinfo_throttle') as info_throttle, \
                patch('flexbe_core.core.operatable_state_machine.Logger.logwarn_throttle') as warn_throttle:
            OperatableStateMachine._mirror_structure_callback(sm, type('_Msg', (), {'data': 7})())
            self.assertEqual([topic for topic, _ in published], [Topics._MIRROR_STRUCTURE_TOPIC])
            self.assertEqual(enabled, [True])
            self.assertTrue(sm._inner_sync_request)
            localinfo.assert_called_once()

            sm._inner_sync_request = False
            OperatableStateMachine._mirror_structure_callback(sm, type('_Msg', (), {'data': 8})())
            info_throttle.assert_called_once()

            sm._structure = None
            OperatableStateMachine._mirror_structure_callback(sm, type('_Msg', (), {'data': 7})())
            warn_throttle.assert_called_once()

    def test_operatable_state_machine_on_enter_resets_cached_outcomes(self):
        """Entering should clear cached outcome and exception bookkeeping before delegating upward."""
        sm = object.__new__(OperatableStateMachine)
        sm._last_outcome = 'done'
        sm._last_exception = RuntimeError('boom')
        sm._last_requested_outcome = 'done'

        with patch('flexbe_core.core.operatable_state_machine.PreemptableStateMachine.on_enter') as parent_on_enter:
            OperatableStateMachine.on_enter(sm, userdata='userdata')

        self.assertIsNone(sm._last_outcome)
        self.assertIsNone(sm._last_exception)
        self.assertIsNone(sm._last_requested_outcome)
        parent_on_enter.assert_called_once_with('userdata')

    def test_operatable_state_machine_on_exit_preempts_current_state_and_clears_requested_outcome(self):
        """Exiting with an active child should preempt it, notify the mirror, and reset state flags."""
        published = []
        child_exit_userdata = []

        class _Child:

            def __init__(self):
                self.name = 'child'
                self.path = '/sm/child'
                self.input_keys = ['value']
                self.output_keys = ['value']
                self._exited = False
                self._last_outcome = None
                self._entering = False

            def on_exit(self, userdata):
                child_exit_userdata.append(userdata)

            def _publish_outcome(self, outcome):
                published.append(('child', outcome))

        sm = object.__new__(OperatableStateMachine)
        sm._name = 'sm'
        sm._parent = type('_Parent', (), {'path': ''})()
        sm._current_state = _Child()
        sm._userdata = UserData()
        sm._userdata.value = 1
        sm._remappings = {'child': {'value': 'value'}}
        sm._state_id = 99
        sm._last_outcome = None
        sm._last_requested_outcome = 'done'
        sm._pub = type('_Pub', (), {'publish': staticmethod(lambda topic, msg: published.append((topic, msg)))})()
        sm._exited = False
        sm._entering = False

        with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'):
            OperatableStateMachine.on_exit(sm)

        self.assertEqual(len(child_exit_userdata), 1)
        self.assertEqual(published[0], ('child', State._preempted_name))
        self.assertEqual(published[1][0], Topics._OUTCOME_TOPIC)
        self.assertEqual(published[2][0], Topics._DEBUG_TOPIC)
        self.assertEqual(published[3][0], Topics._OUTCOME_REQUEST_TOPIC)
        self.assertIsNone(sm._current_state)
        self.assertIsNone(sm._last_requested_outcome)
        self.assertTrue(sm._exited)
        self.assertTrue(sm._entering)

    def test_process_sync_request_logs_error_without_pending_flag(self):
        """Sync processing without a pending flag should emit an error instead of publishing."""
        sm = object.__new__(OperatableStateMachine)
        sm._inner_sync_request = False
        sm._name = 'sm'

        with patch('flexbe_core.core.operatable_state_machine.Logger.error') as log_error:
            OperatableStateMachine.process_sync_request(sm)

        log_error.assert_called_once()

    def test_process_sync_request_logs_invalid_requested_outcome_without_republish(self):
        """Sync resend should log and skip outcome publication when the remembered outcome is invalid."""
        sm = object.__new__(OperatableStateMachine)
        published = []

        class _Pub:

            def publish(self, topic, msg):
                published.append((topic, msg))

        invalid = type(
            '_State',
            (),
            {
                '_last_requested_outcome': 'missing',
                'outcomes': ['done'],
                'state_id': 20,
                'path': '/root/child',
            },
        )()

        sm._inner_sync_request = True
        sm._pub = _Pub()
        sm._last_deep_states_list = [invalid]
        sm.get_latest_status = lambda: 'sync-status'

        with patch('flexbe_core.core.operatable_state_machine.Logger.localinfo'), \
                patch('flexbe_core.core.operatable_state_machine.Logger.localerr') as local_error:
            OperatableStateMachine.process_sync_request(sm)

        self.assertFalse(sm._inner_sync_request)
        self.assertEqual(
            [topic for topic, _msg in published],
            [Topics._MIRROR_SYNC_TOPIC, Topics._CMD_FEEDBACK_TOPIC],
        )
        local_error.assert_called_once()

    def test_state_map_distinguishes_status_only_from_first_real_outcome(self):
        """Status-only hashes use the raw state id while real outcomes keep the +1 offset."""
        state = _PassiveState('active')
        state._state_id = 0x1200

        self.assertEqual(StateMap.hash(state, None), state.state_id)
        self.assertEqual(StateMap.unhash(state.state_id), (state.state_id, None))
        self.assertEqual(StateMap.hash(state, 0), state.state_id + 1)
        self.assertEqual(StateMap.unhash(state.state_id + 1), (state.state_id, 0))

    def test_state_map_rejects_duplicate_explicit_state_ids(self):
        """Explicitly assigned state ids should not silently overwrite earlier entries."""
        state_map = StateMap()
        first = _PassiveState('first')
        second = _PassiveState('second')
        first._state_id = 0x2200
        second._state_id = 0x2200

        state_map.add_state('/first', first)

        with patch('flexbe_core.core.state_map.Logger.error') as log_error:
            with self.assertRaises(KeyError):
                state_map.add_state('/second', second)

        log_error.assert_called_once()
        self.assertIs(state_map.get_state(first.state_id), first)

    def test_state_map_get_path_hash_retries_collisions_and_reports_missing_state(self):
        """Path hash lookup should retry repeated hashes and log when a state id is absent."""
        state_map = StateMap()
        state_map._state_map = {0x2200: object()}
        state_map._num_collision_processed = 2

        with patch.object(StateMap, '_hash_path', side_effect=[0x2100, 0x2200]):
            self.assertEqual(state_map.get_path_hash('/path'), 0x2200)

        self.assertIn('resolved 2 state map id collisions', str(state_map))
        self.assertIsNone(state_map[123])
        with patch('flexbe_core.core.state_map.Logger.error') as log_error:
            self.assertIsNone(state_map.get_state(999))
        log_error.assert_called_once()

    def test_state_machine_reuses_userdata_wrapper_between_ticks(self):
        """Reuse the state machine userdata wrapper across repeated ticks."""
        sm = _DoneTrackingStateMachine()
        state = _WrapperTrackingState()
        with sm:
            StateMachine.add('state', state, transitions={'done': 'done'}, remapping={'value': 'value'})

        initial_userdata = UserData()
        initial_userdata.value = 0

        self.assertIsNone(sm.execute(initial_userdata))
        self.assertEqual(sm.execute(initial_userdata), 'done')
        self.assertEqual(len(state.wrapper_ids), 2)
        self.assertEqual(state.wrapper_ids[0], state.wrapper_ids[1])

    def test_concurrency_container_preserves_shared_userdata_between_states(self):
        """Concurrency states should observe and update the same shared userdata reference."""
        cc = ConcurrencyContainer(outcomes=['done'],
                                  conditions=[('done', [('first', 'done'), ('second', 'done')])])
        first = _WrapperTrackingState()
        second = _WrapperTrackingState()
        with cc:
            OperatableStateMachine.add('first', first, transitions={'done': 'done'}, autonomy={'done': 0},
                                       remapping={'value': 'value'})
            OperatableStateMachine.add('second', second, transitions={'done': 'done'}, autonomy={'done': 0},
                                       remapping={'value': 'value'})

        cc._userdata = UserData()
        cc._userdata.value = 0
        cc._entering = False
        cc._exited = False
        cc._returned_outcomes = {}
        cc._current_state = []
        cc._is_controlled = False
        cc._force_transition = False
        cc._last_requested_outcome = None
        cc._last_outcome = None
        cc._inner_sync_request = False
        cc._pub = type('_Pub', (), {'publish': staticmethod(lambda *args, **kwargs: None)})()

        RosState._current_execution_time_ns = 0
        self.assertIsNone(cc._execute_current_state())
        self.assertEqual(cc._userdata.value, 2)
        RosState._current_execution_time_ns = 0
        self.assertEqual(cc._execute_current_state(), 'done')
        self.assertEqual(cc._userdata.value, 4)
        self.assertEqual(len(first.wrapper_ids), 2)
        self.assertEqual(len(second.wrapper_ids), 2)

    def test_ros_state_target_wakeup_uses_cached_nanoseconds(self):
        """Compute target wakeup time from cached execution nanoseconds without clock subtraction."""
        original_node = RosState._node
        try:
            RosState._node = _FakeNode(1_500_000_000)
            state = RosState(outcomes=['done'])
            state._desired_period_ns = 1_000_000_000
            state._last_execution = _FakeClockTime(500_000_000)
            state._last_execution_ns = 500_000_000
            self.assertEqual(state.target_wakeup_ns, 1_500_000_000)
            self.assertEqual(state._last_execution.nanoseconds, 500_000_000)
        finally:
            RosState._node = original_node

    def test_ros_state_target_wakeup_preserves_no_sleep_sentinel(self):
        """States that have not executed yet should still signal the no-sleep sentinel."""
        state = RosState(outcomes=['done'])
        state._last_execution_ns = None
        self.assertEqual(state.target_wakeup_ns, -1)

    def test_ros_state_machine_wait_sleeps_only_for_positive_remaining_time(self):
        """ROS state machines should only sleep when the requested wakeup is still in the future."""
        sm = object.__new__(RosStateMachine)
        sleep_calls = []

        class _Clock:

            class _Now:
                nanoseconds = 100

            @staticmethod
            def now():
                return _Clock._Now()

            @staticmethod
            def sleep_for(duration, context=None):
                sleep_calls.append((duration.nanoseconds, context))

        sm._node = type('_Node', (), {'get_clock': staticmethod(lambda: _Clock())})()

        sm.wait(None)
        sm.wait(90)
        sm.wait(150, context='ctx')

        self.assertEqual(sleep_calls, [(50, 'ctx')])

    def test_ros_state_machine_control_toggle_fans_out_to_children(self):
        """ROS control enable and disable should toggle once and recurse into child states."""
        sm = object.__new__(RosStateMachine)
        child_calls = []

        class _Child:

            def _enable_ros_control(self):
                child_calls.append('enable')

            def _disable_ros_control(self):
                child_calls.append('disable')

        sm._is_controlled = False
        sm._states = [_Child(), _Child()]

        sm._enable_ros_control()
        sm._enable_ros_control()
        sm._disable_ros_control()
        sm._disable_ros_control()

        self.assertEqual(child_calls, ['enable', 'enable', 'disable', 'disable'])

    def test_state_machine_target_wakeup_preserves_entering_sentinel(self):
        """Entering state machines should preserve the historical negative no-sleep sentinel."""
        sm = StateMachine(outcomes=['done'])
        sm._entering = True
        self.assertEqual(sm.target_wakeup_ns, -1)

    def test_ros_state_logs_custom_desired_rate_once(self):
        """Repeated states with the same custom rate should not spam localinfo logs."""
        original_logged_rates = RosState._logged_desired_rates
        RosState._logged_desired_rates = set()
        fake_proxy = _FakeProxyFactory()
        try:
            with patch('flexbe_core.core.ros_state.ProxyPublisher', return_value=fake_proxy), \
                    patch('flexbe_core.core.ros_state.ProxySubscriberCached', return_value=fake_proxy), \
                    patch('flexbe_core.core.ros_state.Logger.localinfo') as localinfo:
                RosState(outcomes=['done'], desired_rate=42.0)
                RosState(outcomes=['done'], desired_rate=42.0)
                RosState(outcomes=['done'], desired_rate=84.0)

            self.assertEqual(localinfo.call_count, 2)
        finally:
            RosState._logged_desired_rates = original_logged_rates

    def test_ros_state_owns_command_feedback_publisher(self):
        """Shared feedback publisher should be created once at RosState level for layered states."""
        _FakeProxyPublisher.reset()
        _FakeProxySubscriber.reset()

        class _FeedbackState(EventState):

            def __init__(self):
                super().__init__(outcomes=['done'])

            def execute(self, userdata):
                return None

        with patch('flexbe_core.core.ros_state.ProxyPublisher', _FakeProxyPublisher), \
                patch('flexbe_core.core.ros_state.ProxySubscriberCached', _FakeProxySubscriber):
            state = _FeedbackState()
            state._enable_ros_control()
            state._disable_ros_control()

        self.assertEqual(_FakeProxyPublisher.created_topics.count(Topics._CMD_FEEDBACK_TOPIC), 1)
        self.assertEqual(_FakeProxyPublisher.removed_topics.count(Topics._CMD_FEEDBACK_TOPIC), 1)

    def test_preemptable_state_uses_global_preempt_subscription_from_parent_machine(self):
        """Nested states should not subscribe locally when a parent machine owns preempt handling."""
        _FakeProxyPublisher.reset()
        _FakeProxySubscriber.reset()

        class _PreemptState(EventState):

            def __init__(self):
                super().__init__(outcomes=['done'])

            def execute(self, userdata):
                return None

        with patch('flexbe_core.core.ros_state.ProxyPublisher', _FakeProxyPublisher), \
                patch('flexbe_core.core.ros_state.ProxySubscriberCached', _FakeProxySubscriber):
            state = _PreemptState()
            state.set_parent(_FakeGlobalPreemptParent())
            state.set_name('state')
            state._enable_ros_control()
            state._disable_ros_control()

        subscribed_topics = [topic for topic, _inst_id in _FakeProxySubscriber.subscribed]
        unsubscribed_topics = [topic for topic, _inst_id in _FakeProxySubscriber.unsubscribed]
        self.assertNotIn(Topics._CMD_PREEMPT_TOPIC, subscribed_topics)
        self.assertNotIn(Topics._CMD_PREEMPT_TOPIC, unsubscribed_topics)

    def test_preemptable_state_subscribes_locally_without_parent_machine_handler(self):
        """Standalone controlled states should still own their local preempt subscription."""
        _FakeProxyPublisher.reset()
        _FakeProxySubscriber.reset()

        class _PreemptState(EventState):

            def __init__(self):
                super().__init__(outcomes=['done'])

            def execute(self, userdata):
                return None

        with patch('flexbe_core.core.ros_state.ProxyPublisher', _FakeProxyPublisher), \
                patch('flexbe_core.core.ros_state.ProxySubscriberCached', _FakeProxySubscriber):
            state = _PreemptState()
            state._enable_ros_control()
            state._disable_ros_control()

        subscribed_topics = [topic for topic, _inst_id in _FakeProxySubscriber.subscribed]
        unsubscribed_topics = [topic for topic, _inst_id in _FakeProxySubscriber.unsubscribed]
        self.assertIn(Topics._CMD_PREEMPT_TOPIC, subscribed_topics)
        self.assertIn(Topics._CMD_PREEMPT_TOPIC, unsubscribed_topics)

    def test_preemptable_state_machine_consumes_and_acknowledges_controlled_preempt(self):
        """Global machine preempt callback should consume the message and publish feedback once."""
        sm = object.__new__(PreemptableStateMachine)
        removed = []
        feedback = []

        class _Sub:

            def has_msg(self, topic):
                return topic == Topics._CMD_PREEMPT_TOPIC

            def remove_last_msg(self, topic, clear_buffer=False):  # noqa: ARG002
                removed.append(topic)

        class _Pub:

            def publish(self, topic, msg):
                feedback.append((topic, msg.command))

        sm._sub = _Sub()
        sm._pub = _Pub()
        sm._is_controlled = True
        sm._name = 'sm'
        PreemptableStateMachine._preempt_cb(sm, None)

        self.assertTrue(PreemptableState.preempt)
        self.assertEqual(removed, [Topics._CMD_PREEMPT_TOPIC])
        self.assertEqual(feedback, [(Topics._CMD_FEEDBACK_TOPIC, 'preempt')])
        PreemptableState.preempt = False

    def test_preemptable_state_machine_spin_rejects_unhandled_transition_command(self):
        """Spin should toss buffered transition commands that no state machine layer handled."""
        sm = object.__new__(PreemptableStateMachine)
        removed = []
        feedback = []
        command = type('_Command', (), {'target': 77, 'outcome': 0})()

        class _Sub:

            def __init__(self):
                self.buffered = True

            def peek_if_buffered(self, topic):
                if self.buffered and topic == Topics._CMD_TRANSITION_TOPIC:
                    return command
                return None

            def get_from_buffer(self, topic):
                removed.append(topic)
                self.buffered = False
                return command

        class _Pub:

            def publish(self, topic, msg):
                feedback.append((topic, msg.command, list(msg.args)))

        class _Node:

            class _Clock:

                class _Now:
                    nanoseconds = 123

                def now(self):
                    return _Node._Clock._Now()

            def get_clock(self):
                return _Node._Clock()

        sm._name = 'sm'
        sm._node = _Node()
        sm._sub = _Sub()
        sm._pub = _Pub()
        sm._status_lock = threading.Lock()
        sm._last_deep_states_list = ()
        sm._inner_sync_request = False
        sm.get_deep_states = lambda: ()
        sm.execute = lambda userdata=None: 'done'
        sm.wait = lambda target_wakeup_ns=None: None

        with patch('flexbe_core.core.preemptable_state_machine.rclpy.ok', return_value=True), \
                patch('flexbe_core.core.preemptable_state_machine.Logger.loginfo'), \
                patch('flexbe_core.core.preemptable_state_machine.Logger.localinfo'), \
                patch('flexbe_core.core.preemptable_state_machine.Logger.logerr'):
            outcome = PreemptableStateMachine.spin(sm)

        self.assertEqual(outcome, 'done')
        self.assertEqual(removed, [Topics._CMD_TRANSITION_TOPIC])
        self.assertEqual(
            feedback,
            [(Topics._CMD_FEEDBACK_TOPIC, 'transition', ['invalid', '77'])],
        )


if __name__ == '__main__':
    unittest.main()
