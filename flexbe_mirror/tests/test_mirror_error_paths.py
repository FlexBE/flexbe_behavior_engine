#!/usr/bin/env python3

# Copyright 2026  Christopher Newport University
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


"""Unit tests for typed mirror sync and transition error paths."""

import threading
import types
import unittest
from collections import deque
from unittest.mock import patch

from flexbe_core.core import PreemptableState, State, SyncError, TransitionError
from flexbe_core.core import StateMap
from flexbe_core.core.topics import Topics

import flexbe_mirror.behavior_mirror_sm as behavior_mirror_sm
from flexbe_mirror.flexbe_mirror import FlexbeMirror
from flexbe_mirror.mirror_concurrency_container import MirrorConcurrencyContainer
from flexbe_mirror.mirror_state import MirrorState
from flexbe_mirror.mirror_state_machine import MirrorStateMachine

from flexbe_msgs.msg import BEStatus, BehaviorSync


class TestMirrorErrorPaths(unittest.TestCase):
    """Validate mirror methods raise typed errors in expected edge cases."""

    @staticmethod
    def _noop(*args, **kwargs):
        return None

    def _make_mirror(self):
        mirror = FlexbeMirror.__new__(FlexbeMirror)
        mirror._sync_lock = threading.Lock()
        mirror._timing_event = threading.Event()
        mirror._shutdown_requested = False
        mirror._last_onboard_mismatch_sig = None
        mirror._last_mirror_mismatch_sig = None
        mirror._start_requested = False
        mirror._system_clock = types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=1)
        )
        mirror._wait_timeout_sec = 4.0
        mirror._wait_poll_sec = 0.2
        mirror._wait_stopping = self._noop
        mirror._wait_stop_running = self._noop
        mirror._handle_execution_exception = self._noop
        mirror.get_elapsed_str = lambda _start_time: 'elapsed'
        mirror._pending_terminal_status_code = None
        mirror._pending_terminal_status_behavior_id = BehaviorSync.INVALID
        mirror._pending_terminal_status_args = []
        mirror._mirror_sync_warning_active = False
        mirror._pending_start_behavior_id = BehaviorSync.INVALID
        mirror._pending_start_args = []
        mirror._soft_stop_requested = False
        mirror._soft_stop_thread = None
        mirror._starting = False
        mirror._stopping = False
        mirror._running = False
        mirror._active_id = BehaviorSync.INVALID
        mirror._last_obe_status = None
        mirror._last_stop_behavior_id = BehaviorSync.INVALID
        mirror._last_stop_status_code = None
        mirror._notify_state_change = lambda: FlexbeMirror._notify_state_change(mirror)
        mirror._mirror_status_pub = types.SimpleNamespace(publish=self._noop)
        return mirror

    @staticmethod
    def _capture_statuses():
        statuses = []

        def publish(msg):
            statuses.append({
                'code': msg.code,
                'behavior_id': msg.behavior_id,
                'args': list(msg.args),
            })

        return statuses, types.SimpleNamespace(publish=publish)

    def test_mirror_state_machine_waits_on_shared_outcome_event_in_idle_loop(self):
        """Idle mirror spin should reuse the shared outcome subscriber instead of creating another one."""
        event_waits = []
        subscribed = []
        unsubscribed = []

        class _FakeEvent:

            def clear(self):
                return None

            def set(self):  # noqa: A003
                return None

            def wait(self, timeout=None):
                event_waits.append(timeout)
                PreemptableState.preempt = True
                return True

        class _FakeProxySubscriber:

            def subscribe(self, topic, _msg_type, callback=None, inst_id=None, **_kwargs):
                subscribed.append((topic, inst_id, callback is not None))
                self._callback = callback

            def has_buffered(self, _topic):
                return False

            def get_from_buffer(self, _topic):
                raise AssertionError('idle loop should not pull buffered messages')

            def unsubscribe_topic(self, topic, inst_id=-1):
                unsubscribed.append((topic, inst_id))

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm.id = 7
        sm._state_id = 1
        sm._parent = types.SimpleNamespace(path='', path_segments=())
        sm._status_lock = threading.Lock()
        sm._current_state = types.SimpleNamespace(name='leaf_mirror', state_id=2)
        sm._last_outcome = None
        sm._last_deep_states_list = ['stable']
        sm._status_event_callback = None
        sm._outcome_sub = _FakeProxySubscriber()
        sm._total_loop_count = 0
        sm.get_deep_states = lambda: ['stable']
        sm._execute_current_state_mirror = lambda _userdata: None

        PreemptableState.preempt = False
        try:
            with patch('flexbe_mirror.mirror_state_machine.Event', _FakeEvent), \
                    patch('flexbe_mirror.mirror_state_machine.rclpy.ok', side_effect=[True, True, True, False]), \
                    patch.multiple('flexbe_mirror.mirror_state_machine.Logger',
                                   check_local_enabled=self._noop,
                                   localinfo=self._noop,
                                   localwarn=self._noop,
                                   logwarn=self._noop,
                                   logerr=self._noop,
                                   localerr=self._noop):
                outcome = sm.spin(types.SimpleNamespace(nanoseconds=1), {})
        finally:
            PreemptableState.preempt = False

        self.assertEqual(PreemptableState.preempt, False)
        self.assertIn(outcome, (None, State._preempted_name))
        self.assertEqual(event_waits, [0.05])
        self.assertEqual(len(subscribed), 1)
        self.assertEqual(subscribed[0][0], Topics._OUTCOME_TOPIC)
        self.assertTrue(subscribed[0][2])
        self.assertEqual(len(unsubscribed), 1)
        self.assertEqual(unsubscribed[0][0], Topics._OUTCOME_TOPIC)

    def test_mirror_state_publish_update_deduplicates_repeated_target_ids(self):
        """Deduplicate UI update publications when the target id does not change."""
        published = []
        MirrorState._pub = types.SimpleNamespace(publish=lambda topic, msg: published.append((topic, msg.data)))
        MirrorState._last_target_id = None

        MirrorState.publish_update(10)
        MirrorState.publish_update(10)
        MirrorState.publish_update(11)

        self.assertEqual(
            published,
            [
                (Topics._BEHAVIOR_UPDATE_TOPIC, 10),
                (Topics._BEHAVIOR_UPDATE_TOPIC, 11),
            ],
        )

    def test_mirror_state_execute_mirror_consumes_matching_outcome_and_handles_invalid_index(self):
        """Process matching outcome messages and fail softly on invalid indexes."""
        MirrorState._pub = types.SimpleNamespace(publish=self._noop)
        state = MirrorState('leaf_mirror', '/root/leaf_mirror', ['done'], {})
        state._state_id = 77

        with patch.object(MirrorState, 'publish_update', self._noop):
            MirrorState._last_state_id = state.state_id
            MirrorState._last_state_outcome = 0
            outcome = state.execute_mirror({})

        self.assertEqual(outcome, 'done')
        self.assertIsNone(MirrorState._last_state_id)
        self.assertIsNone(MirrorState._last_state_outcome)

        with patch('flexbe_mirror.mirror_state.Logger.localerr') as localerr:
            invalid = state.on_exit_mirror({}, 5)
        self.assertIsNone(invalid)
        self.assertGreaterEqual(localerr.call_count, 2)

        preempted = state.on_exit_mirror({}, StateMap._MAX_OUTCOME)
        self.assertEqual(preempted, State._preempted_name)

    def test_mirror_concurrency_container_tracks_active_children_and_returned_outcomes(self):
        """Concurrency mirror execution should retain active children and mark returned outcomes."""
        published = []

        class _DoneChild:

            def __init__(self):
                self.name = 'done_child'
                self._entering = False
                self._last_execution = object()

            def execute_mirror(self, userdata):
                return 'done'

        class _ActiveChild:

            def __init__(self):
                self.name = 'active_child'
                self._entering = False
                self._last_execution = object()

            def execute_mirror(self, userdata):
                return None

        cc = object.__new__(MirrorConcurrencyContainer)
        cc._states = [_DoneChild(), _ActiveChild()]
        cc._state_id = 21
        cc._returned_outcomes = {}
        cc._current_state = None
        cc._userdata = object()
        cc._last_outcome = 'old'
        cc._entering = True
        cc._transitions = {}

        with patch.object(MirrorState, 'publish_update', side_effect=lambda target_id: published.append(target_id)):
            result = cc.execute_mirror({})

        self.assertIsNone(result)
        self.assertEqual(cc._returned_outcomes, {'done_child': 'done'})
        self.assertEqual([state.name for state in cc._current_state], ['active_child'])
        self.assertFalse(cc._entering)
        self.assertEqual(published[0], cc.state_id)

    def test_mirror_concurrency_container_exit_preempts_remaining_children_and_reports_deep_states(self):
        """Concurrency mirror exit should preempt unfinished children and report deep states for active lists."""
        published = []
        child_exits = []

        class _Child:

            def __init__(self, name, locked=False):
                self.name = name
                self._entering = False
                self._last_execution = object()
                self.state_id = 100 if name == 'first' else 101
                self.locked = locked

            def on_exit_mirror(self, userdata, desired_outcome):
                child_exits.append((self.name, desired_outcome))

            def is_locked(self):
                return False

        cc = object.__new__(MirrorConcurrencyContainer)
        cc._state_id = 55
        cc._states = [_Child('first'), _Child('second')]
        cc._returned_outcomes = {'first': 'done'}
        cc._current_state = cc._states[:]
        cc._entering = False
        cc._last_outcome = None
        cc._outcomes = ['finished']
        cc._name = 'cc_mirror'

        with patch.object(MirrorState, 'publish_update', side_effect=lambda target_id: published.append(target_id)):
            outcome = cc.on_exit_mirror({}, 0)

        self.assertEqual(outcome, 'finished')
        self.assertEqual(child_exits, [('first', -1), ('second', -1)])
        self.assertEqual(published[-1], cc.state_id + 255)
        self.assertIsNone(cc._current_state)
        self.assertEqual(cc._returned_outcomes, {})

        cc._current_state = [cc._states[0]]
        deep_states = cc.get_deep_states()
        self.assertEqual(deep_states, [cc, cc._states[0]])

        cc._current_state = types.SimpleNamespace(name='bad', state_id=999)
        with patch('flexbe_mirror.mirror_concurrency_container.Logger.localerr') as localerr:
            with self.assertRaises(TypeError):
                cc.get_deep_states()
        self.assertGreaterEqual(localerr.call_count, 2)

    def test_mirror_state_machine_reuses_cached_latest_status(self):
        """Mirror status message should be reused while active-state snapshot is unchanged."""
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm.id = 9
        sm._status_lock = threading.Lock()
        active_state = types.SimpleNamespace(
            _last_outcome=None,
            _outcomes=[],
            state_id=21
        )
        sm._last_deep_states_list = [active_state]
        sm._cached_status_msg = None
        sm._cached_status_states = None
        sm._cached_status_behavior_id = BehaviorSync.INVALID

        with patch('flexbe_mirror.mirror_state_machine.StateMap.hash',
                   side_effect=lambda _state, outcome_index: 100 if outcome_index is None else 100 + outcome_index) as hash_mock:
            first = sm.get_latest_status()
            second = sm.get_latest_status()

        self.assertIs(first, second)
        self.assertEqual(first.behavior_id, 9)
        self.assertEqual(list(first.current_state_checksums), [100])
        hash_mock.assert_called_once_with(active_state, None)

        sm._last_deep_states_list = [active_state, None]
        with patch('flexbe_mirror.mirror_state_machine.StateMap.hash',
                   side_effect=lambda _state, outcome_index: 200 if outcome_index is None else 200 + outcome_index) as hash_mock:
            third = sm.get_latest_status()

        self.assertIsNot(third, first)
        self.assertEqual(list(third.current_state_checksums), [200])
        hash_mock.assert_called_once_with(active_state, None)

    def test_mirror_state_machine_get_latest_status_logs_invalid_outcomes_and_empty_snapshots(self):
        """Latest-status serialization should ignore invalid outcomes and handle missing active states."""
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm.id = 9
        sm._status_lock = threading.Lock()
        active_state = types.SimpleNamespace(
            _last_outcome='missing',
            _outcomes=['done'],
            state_id=21,
        )
        sm._last_deep_states_list = [active_state]
        sm._cached_status_msg = None
        sm._cached_status_states = None
        sm._cached_status_behavior_id = BehaviorSync.INVALID

        with patch('flexbe_mirror.mirror_state_machine.Logger.localerr') as localerr, \
                patch('flexbe_mirror.mirror_state_machine.StateMap.hash', return_value=123) as hash_mock:
            msg = sm.get_latest_status()

        self.assertEqual([123], list(msg.current_state_checksums))
        hash_mock.assert_called_once_with(active_state, None)
        localerr.assert_called_once()

        sm._last_deep_states_list = None
        sm._cached_status_msg = None
        sm._cached_status_states = None
        with patch('flexbe_mirror.mirror_state_machine.Logger.localinfo') as localinfo:
            empty_msg = sm.get_latest_status()
        self.assertEqual([], list(empty_msg.current_state_checksums))
        localinfo.assert_called_once()

    def test_mirror_state_machine_defers_premature_top_level_outcome(self):
        """Top-level outcome should be retained until the root child path completes."""

        class _FakeEvent:

            def clear(self):
                return None

            def set(self):  # noqa: A003
                return None

            def wait(self, timeout=None):
                raise AssertionError('deferred top-level outcome should complete without waiting')

        class _FakeProxySubscriber:

            def __init__(self):
                self._buffer = [types.SimpleNamespace(data=StateMap.hash(sm, 0))]

            def enable_buffer(self, _topic):
                return None

            def subscribe(self, _topic, _msg_type, callback=None, inst_id=None, **_kwargs):
                self._callback = callback

            def has_buffered(self, _topic):
                return bool(self._buffer)

            def get_from_buffer(self, _topic):
                return self._buffer.pop(0)

            def unsubscribe_topic(self, _topic, inst_id=-1):
                return None

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm.id = 7
        sm._state_id = 1
        sm._parent = types.SimpleNamespace(path='', path_segments=())
        sm._status_lock = threading.Lock()
        sm._current_state = types.SimpleNamespace(
            name='leaf_mirror',
            state_id=2,
            _entering=False,
            execute_mirror=lambda _userdata: 'done'
        )
        sm._last_outcome = None
        sm._pending_outcomes = deque(maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = None
        sm._last_deep_states_list = ['stable']
        sm._status_event_callback = None
        sm._outcome_sub = _FakeProxySubscriber()
        sm._total_loop_count = 0
        sm._transitions = {'leaf_mirror': {'done': 'finished'}}
        sm._labels = {'finished': None}
        sm._outcomes = ['complete']
        sm.get_deep_states = lambda: ['stable']

        PreemptableState.preempt = False
        try:
            with patch('flexbe_mirror.mirror_state_machine.Event', _FakeEvent), \
                    patch('flexbe_mirror.mirror_state_machine.StateMap.unhash', return_value=(1, 0)), \
                    patch('flexbe_mirror.mirror_state_machine.rclpy.ok', side_effect=[True, True, True, False]), \
                    patch('flexbe_mirror.mirror_state.MirrorState.publish_update', self._noop), \
                    patch.multiple('flexbe_mirror.mirror_state_machine.Logger',
                                   check_local_enabled=self._noop,
                                   localinfo=self._noop,
                                   localwarn=self._noop,
                                   logwarn=self._noop,
                                   logerr=self._noop,
                                   localerr=self._noop):
                outcome = sm.spin(types.SimpleNamespace(nanoseconds=1), {})
        finally:
            PreemptableState.preempt = False

        self.assertEqual(outcome, 'complete')
        self.assertEqual([], list(sm._pending_outcomes))
        self.assertIsNone(sm._current_state)
        self.assertEqual(sm._last_outcome, 'complete')

    def test_mirror_state_machine_ignores_legacy_zero_outcomes_during_new_run(self):
        """Legacy raw-zero barrier messages should be ignored without gating outcome processing."""

        class _FakeEvent:

            def clear(self):
                return None

            def set(self):  # noqa: A003
                return None

            def wait(self, timeout=None):
                raise AssertionError('barrier-gated outcome processing should not idle-wait')

        class _FakeProxySubscriber:

            def __init__(self):
                self._buffer = [
                    types.SimpleNamespace(data=111),
                    types.SimpleNamespace(data=0),
                    types.SimpleNamespace(data=222),
                ]

            def enable_buffer(self, _topic):
                return None

            def subscribe(self, _topic, _msg_type, callback=None, inst_id=None, **_kwargs):
                self._callback = callback

            def has_buffered(self, _topic):
                return bool(self._buffer)

            def get_from_buffer(self, _topic):
                return self._buffer.pop(0)

            def unsubscribe_topic(self, _topic, inst_id=-1):
                return None

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm.id = 7
        sm._state_id = 1
        sm._parent = types.SimpleNamespace(path='', path_segments=())
        sm._status_lock = threading.Lock()
        sm._current_state = types.SimpleNamespace(
            name='leaf_mirror',
            state_id=2,
            _entering=False,
            execute_mirror=lambda _userdata: 'done'
        )
        sm._last_outcome = None
        sm._pending_outcomes = deque(maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = None
        sm._last_deep_states_list = ['stable']
        sm._status_event_callback = None
        sm._outcome_sub = _FakeProxySubscriber()
        sm._total_loop_count = 0
        sm._transitions = {'leaf_mirror': {'done': 'finished'}}
        sm._labels = {'finished': None}
        sm._outcomes = ['complete']
        sm.get_deep_states = lambda: ['stable']

        PreemptableState.preempt = False
        try:
            with patch('flexbe_mirror.mirror_state_machine.Event', _FakeEvent), \
                    patch('flexbe_mirror.mirror_state_machine.StateMap.unhash', side_effect=[(1, 0), (1, 0)]), \
                    patch('flexbe_mirror.mirror_state_machine.rclpy.ok', side_effect=[True, True, True, True, False]), \
                    patch('flexbe_mirror.mirror_state.MirrorState.publish_update', self._noop), \
                    patch.multiple('flexbe_mirror.mirror_state_machine.Logger',
                                   check_local_enabled=self._noop,
                                   localinfo=self._noop,
                                   localwarn=self._noop,
                                   logwarn=self._noop,
                                   logerr=self._noop,
                                   localerr=self._noop):
                outcome = sm.spin(types.SimpleNamespace(nanoseconds=1), {})
        finally:
            PreemptableState.preempt = False

        self.assertEqual(outcome, 'complete')
        self.assertEqual([], list(sm._pending_outcomes))
        self.assertIsNone(sm._current_state)
        self.assertEqual(sm._last_outcome, 'complete')

    def test_mirror_state_machine_stops_draining_buffered_outcomes_on_preempt(self):
        """Preempt should stop buffered outcome draining immediately."""

        class _FakeProxySubscriber:

            def __init__(self):
                self._buffer = [
                    types.SimpleNamespace(data=111),
                    types.SimpleNamespace(data=222),
                ]

            def has_buffered(self, _topic):
                return bool(self._buffer)

            def get_from_buffer(self, _topic):
                return self._buffer.pop(0)

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._pending_outcomes = deque(maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = None
        sm._state_id = 1
        sm._current_state = None
        sm.get_deep_states = lambda: []

        PreemptableState.preempt = True
        try:
            with patch('flexbe_mirror.mirror_state_machine.StateMap.unhash',
                       side_effect=AssertionError('preempt should stop before decoding outcomes')):
                drained_any = sm._drain_buffered_outcomes(_FakeProxySubscriber())
        finally:
            PreemptableState.preempt = False

        self.assertFalse(drained_any)
        self.assertEqual([], list(sm._pending_outcomes))

    def test_internal_mirror_state_machine_defers_premature_container_outcome(self):
        """Internal container outcomes should be consumed once the child path completes."""
        child = types.SimpleNamespace(name='leaf_mirror', state_id=11)
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'branch_mirror'
        sm._state_id = 10
        sm._current_state = child
        sm._last_outcome = None
        sm._pending_outcomes = deque(maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = None
        sm._entering = False
        sm._status_event_callback = None
        sm._execute_current_state_mirror = lambda _userdata: None

        try:
            from flexbe_mirror.mirror_state import MirrorState
            MirrorState._last_state_id = 10
            MirrorState._last_state_outcome = 0

            with patch.multiple('flexbe_mirror.mirror_state_machine.Logger',
                                localwarn=self._noop):
                outcome = sm.execute_mirror({})
        finally:
            from flexbe_mirror.mirror_state import MirrorState
            MirrorState._last_state_id = None
            MirrorState._last_state_outcome = None

        self.assertIsNone(outcome)
        self.assertIs(sm._current_state, child)
        self.assertEqual([(10, 0)], [(sid, out) for sid, out, _seen_at in sm._pending_outcomes])

        del sm._execute_current_state_mirror
        sm._current_state = None
        with patch.object(MirrorStateMachine, 'on_exit_mirror', return_value='done') as exit_mock, \
                patch.multiple('flexbe_mirror.mirror_state_machine.Logger',
                               localwarn=self._noop):
            outcome = MirrorStateMachine.execute_mirror(sm, {})

        self.assertEqual(outcome, 'done')
        exit_mock.assert_called_once_with({}, 0)
        self.assertEqual([], list(sm._pending_outcomes))

    def test_pending_outcome_queue_preserves_fifo_order_per_state(self):
        """Queued deferred outcomes should replay in FIFO order for each state."""
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._state_id = 10
        sm._name = 'branch_mirror'
        sm._pending_outcomes = deque(maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = None
        sm._current_state = None
        sm._status_event_callback = None

        sm._defer_outcome_mirror(10, 1, now_sec=1.0)
        sm._defer_outcome_mirror(10, 2, now_sec=1.1)
        sm._defer_outcome_mirror(10, 3, now_sec=1.2)

        self.assertEqual(1, sm._pop_pending_outcome_for_state(10))
        self.assertEqual(2, sm._pop_pending_outcome_for_state(10))
        self.assertEqual(3, sm._pop_pending_outcome_for_state(10))
        self.assertIsNone(sm._pop_pending_outcome_for_state(10))

    def test_pending_outcome_queue_keeps_unmatched_entries_when_state_is_absent(self):
        """Popping a missing state should leave the queue intact and clear the legacy alias."""
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._state_id = 10
        sm._name = 'branch_mirror'
        sm._pending_outcomes = deque(maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = 99
        sm._current_state = None
        sm._status_event_callback = None

        sm._defer_outcome_mirror(11, 1, now_sec=1.0)
        sm._defer_outcome_mirror(12, 2, now_sec=1.1)

        self.assertIsNone(sm._pop_pending_outcome_for_state(10))
        self.assertIsNone(sm._pending_outcome)
        self.assertEqual([(11, 1), (12, 2)], [(sid, out) for sid, out, _seen_at in sm._pending_outcomes])

    def test_pending_outcome_promotes_when_state_becomes_active(self):
        """Queued outcomes should promote once their state appears on active path."""
        from flexbe_mirror.mirror_state import MirrorState

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm._state_id = 1
        sm._current_state = types.SimpleNamespace(state_id=2)
        sm._pending_outcomes = deque(maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = None
        sm._status_event_callback = None
        sm.get_deep_states = lambda: [types.SimpleNamespace(state_id=1), types.SimpleNamespace(state_id=2)]

        MirrorState._last_state_id = None
        MirrorState._last_state_outcome = None
        try:
            sm._defer_outcome_mirror(3, 7, now_sec=1.0)  # not active yet
            sm._defer_outcome_mirror(2, 8, now_sec=1.1)  # active
            promoted = sm._promote_pending_outcome_if_relevant(now_sec=1.2)
        finally:
            MirrorState._last_state_id = None
            MirrorState._last_state_outcome = None

        self.assertTrue(promoted)
        self.assertEqual(8, sm._pending_outcome)
        self.assertEqual([(3, 7)], [(sid, out) for sid, out, _seen_at in sm._pending_outcomes])

    def test_consume_promoted_top_level_outcome_handles_cleared_and_reprocessed_outcomes(self):
        """Top-level promoted outcomes should clear stale ids and allow reprocessing warnings."""
        from flexbe_mirror.mirror_state import MirrorState

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm._state_id = 7
        sm._parent = types.SimpleNamespace(path='/root_mirror')
        sm._pending_outcome = 99
        sm._current_state = None
        sm._last_outcome = None
        sm.on_exit_mirror = lambda userdata, desired_outcome: ('done', userdata, desired_outcome)

        MirrorState._last_state_id = 7
        MirrorState._last_state_outcome = None
        try:
            self.assertIsNone(sm._consume_promoted_top_level_outcome({}, types.SimpleNamespace(nanoseconds=1)))
            self.assertIsNone(MirrorState._last_state_id)

            MirrorState._last_state_id = 7
            MirrorState._last_state_outcome = 2
            sm._last_outcome = 'done'
            with patch('flexbe_mirror.mirror_state_machine.Logger.localwarn') as localwarn, \
                    patch('flexbe_mirror.mirror_state_machine.MirrorState.publish_update') as publish_update:
                outcome = sm._consume_promoted_top_level_outcome({}, types.SimpleNamespace(nanoseconds=3))
        finally:
            MirrorState._last_state_id = None
            MirrorState._last_state_outcome = None

        self.assertEqual(('done', {}, 2), outcome)
        self.assertIsNone(sm._pending_outcome)
        publish_update.assert_called_once_with(sm.state_id)
        localwarn.assert_called_once()

    def test_active_state_id_set_falls_back_to_self_id_when_deep_state_lookup_fails(self):
        """Deep-state lookup failures should still report the container itself as active."""
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._state_id = 42
        sm.get_deep_states = lambda: (_ for _ in ()).throw(RuntimeError('deep state boom'))

        self.assertEqual({42}, sm._active_state_id_set())

    def test_consume_pending_outcome_waits_for_child_exit_and_uses_on_exit_when_ready(self):
        """Deferred container outcomes should wait for active children and exit once idle."""
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._state_id = 7
        sm._current_state = object()

        self.assertIsNone(MirrorStateMachine._consume_pending_outcome_mirror(sm, {}))

        sm._current_state = None
        sm._pop_pending_outcome_for_state = lambda _state_id: 2
        sm.on_exit_mirror = lambda userdata, desired_outcome: ('done', userdata, desired_outcome)
        self.assertEqual(('done', {}, 2), MirrorStateMachine._consume_pending_outcome_mirror(sm, {}))

        sm._pop_pending_outcome_for_state = lambda _state_id: None
        self.assertIsNone(MirrorStateMachine._consume_pending_outcome_mirror(sm, {}))

    def test_notify_status_event_logs_callback_failures_without_raising(self):
        """Status callback errors should be logged and swallowed."""
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm._state_id = 13
        sm._status_event_callback = lambda _active_states: (_ for _ in ()).throw(RuntimeError('callback boom'))

        with patch('flexbe_mirror.mirror_state_machine.Logger.localwarn') as localwarn:
            sm._notify_status_event(['active'])

        localwarn.assert_called_once()

    def test_mirror_state_machine_on_enter_initializes_state_and_publishes_update(self):
        """Entering the mirror state machine should reset state and publish a container update."""
        initial_state = types.SimpleNamespace(_entering=False)
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._state_id = 11
        sm._states = [initial_state]
        sm.assert_consistent_transitions = self._noop
        sm._pending_outcomes = deque([(1, 2, 3.0)], maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = 9
        sm._last_outcome = 'old'
        sm._current_state = object()
        sm._entering = True

        with patch('flexbe_mirror.mirror_state_machine.MirrorState.publish_update') as publish_update:
            sm.on_enter_mirror({})

        self.assertFalse(sm._entering)
        self.assertIs(sm._current_state, initial_state)
        self.assertTrue(initial_state._entering)
        self.assertIsNone(sm._userdata)
        self.assertEqual([], list(sm._pending_outcomes))
        self.assertIsNone(sm._pending_outcome)
        publish_update.assert_called_once_with(sm.state_id)

    def test_mirror_state_machine_on_exit_handles_valid_preempt_and_invalid_outcomes(self):
        """Exiting the mirror state machine should map preempt and reject invalid outcome indexes."""
        child = types.SimpleNamespace(_entering=False, on_exit_mirror=lambda userdata, desired_outcome: None)
        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm._state_id = 12
        sm._outcomes = ['done']
        sm._current_state = child
        sm._entering = False
        sm._pending_outcomes = deque([(1, 2, 3.0)], maxlen=MirrorStateMachine._PENDING_OUTCOME_MAXLEN)
        sm._pending_outcome = 8

        with patch('flexbe_mirror.mirror_state_machine.MirrorState.publish_update') as publish_update:
            outcome = sm.on_exit_mirror({}, StateMap._MAX_OUTCOME)

        self.assertEqual(State._preempted_name, outcome)
        self.assertIsNone(sm._current_state)
        self.assertTrue(sm._entering)
        self.assertEqual([], list(sm._pending_outcomes))
        self.assertIsNone(sm._pending_outcome)
        publish_update.assert_called_once_with(sm.state_id + 255)

        sm._current_state = child
        sm._entering = False
        with patch('flexbe_mirror.mirror_state_machine.Logger.localerr') as localerr:
            self.assertIsNone(sm.on_exit_mirror({}, 5))
        localerr.assert_called_once()

    def test_mirror_state_machine_destroy_notifies_nested_children(self):
        """Destroy should stop direct mirror states and recurse into nested mirror state machines."""
        child_leaf = MirrorState.__new__(MirrorState)
        child_leaf.on_stop = lambda: setattr(child_leaf, 'stopped', True)
        child_leaf.stopped = False
        nested_leaf = MirrorState.__new__(MirrorState)
        nested_leaf.on_stop = lambda: setattr(nested_leaf, 'stopped', True)
        nested_leaf.stopped = False

        nested = MirrorStateMachine.__new__(MirrorStateMachine)
        nested._states = [nested_leaf]
        nested._current_state = types.SimpleNamespace(state_id=22)

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._name = 'root_mirror'
        sm._state_id = 21
        sm._states = [child_leaf, nested]
        sm._current_state = nested

        with patch('flexbe_mirror.mirror_state_machine.Logger.localinfo'):
            sm.destroy()

        self.assertTrue(child_leaf.stopped)
        self.assertTrue(nested_leaf.stopped)

    def test_mirror_state_machine_get_deep_states_includes_nested_container_and_leaf(self):
        """Deep-state reporting should include the current container path and deepest active leaf."""
        nested = MirrorStateMachine.__new__(MirrorStateMachine)
        nested._current_state = types.SimpleNamespace(state_id=22)

        sm = MirrorStateMachine.__new__(MirrorStateMachine)
        sm._current_state = nested

        self.assertEqual([sm, nested, nested._current_state], sm.get_deep_states())

        sm._current_state = None
        self.assertEqual([sm], sm.get_deep_states())

    def test_wait_stop_running_uses_shared_state_change_event(self):
        """Waiting for a running mirror should block on the shared state-change event."""
        mirror = self._make_mirror()
        mirror._running = True
        mirror._stopping = False
        mirror._starting = True
        mirror._active_id = 5
        mirror._active_thread_start = 77

        class _TrackingEvent:

            def clear(self):
                return None

            def set(self):  # noqa: A003
                return None

            def wait(self, timeout=None):
                mirror._running = False
                return True

        mirror._timing_event = _TrackingEvent()

        with patch('flexbe_mirror.flexbe_mirror.threading.Event',
                   side_effect=AssertionError('shared event should be reused')), \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               loginfo=self._noop):
            mirror._wait_stop_running(types.SimpleNamespace())

    def test_wait_stopping_uses_shared_state_change_event(self):
        """Waiting for stopping completion should block on the shared state-change event."""
        mirror = self._make_mirror()
        mirror._running = False
        mirror._stopping = True
        mirror._starting = True
        mirror._active_id = 5
        mirror._active_thread_start = 77

        class _TrackingEvent:

            def clear(self):
                return None

            def set(self):  # noqa: A003
                return None

            def wait(self, timeout=None):
                mirror._stopping = False
                return True

        mirror._timing_event = _TrackingEvent()

        with patch('flexbe_mirror.flexbe_mirror.threading.Event',
                   side_effect=AssertionError('shared event should be reused')), \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               loginfo=self._noop):
            mirror._wait_stopping(types.SimpleNamespace())

    def test_wait_stop_running_timeout_raises_sync_error(self):
        """Timeout in wait-stop-running should raise SyncError and clear stop flags."""
        mirror = FlexbeMirror.__new__(FlexbeMirror)
        mirror._running = True
        mirror._stopping = True
        mirror._starting = True
        mirror._active_id = 123
        mirror._active_thread_start = 456
        mirror._wait_timeout_sec = 4.0
        mirror._wait_poll_sec = 0.2
        mirror._timing_event = threading.Event()
        mirror.get_elapsed_str = lambda start_time: 'elapsed'

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            loginfo=self._noop,
                            logerr=self._noop):
            with patch('flexbe_mirror.flexbe_mirror.time.monotonic', side_effect=[0.0, 5.0]):
                with self.assertRaises(SyncError):
                    mirror._wait_stop_running(types.SimpleNamespace())

        self.assertFalse(mirror._stopping)
        self.assertFalse(mirror._starting)

    def test_execute_mirror_overlap_raises_transition_error(self):
        """Concurrent execute attempts should raise TransitionError and trigger preempt."""
        mirror = FlexbeMirror.__new__(FlexbeMirror)
        mirror._active_id = 7
        mirror._active_thread_start = 99
        mirror._running = False
        mirror._starting_path = None
        mirror._sm = types.SimpleNamespace(id=7)
        mirror.get_elapsed_str = lambda start_time: 'elapsed'

        PreemptableState.preempt = False
        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            loginfo=self._noop):
            with self.assertRaises(TransitionError):
                mirror._execute_mirror(types.SimpleNamespace(nanoseconds=1))
        self.assertTrue(PreemptableState.preempt)

    def test_restart_mirror_handles_wait_stop_sync_error(self):
        """Ensure SyncError from wait-stop-running is handled inside restart without escaping thread entrypoint."""
        mirror = FlexbeMirror.__new__(FlexbeMirror)
        mirror._sync_lock = threading.Lock()
        mirror._sm = types.SimpleNamespace(id=11)
        mirror._running = True
        mirror._active_id = 11
        mirror._wait_timeout_sec = 4.0
        mirror._wait_poll_sec = 0.2
        mirror._wait_stop_running = lambda *_args, **_kwargs: (_ for _ in ()).throw(SyncError('timeout'))
        mirror._execute_mirror = lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError('must not execute'))
        mirror.get_elapsed_str = lambda _start_time: 'elapsed'

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            loginfo=self._noop):
            mirror._restart_mirror(types.SimpleNamespace(behavior_id=11), types.SimpleNamespace(nanoseconds=1))

    def test_sync_callback_ignores_missing_state_machine_for_active_id(self):
        """Sync callback should not dereference a torn-down state machine."""
        mirror = self._make_mirror()
        mirror._active_id = 23
        mirror._sm = None
        mirror._restart_mirror = lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError('must not restart'))
        mirror._log_exception = self._noop

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            logwarn=self._noop,
                            localinfo=self._noop):
            mirror._sync_callback(types.SimpleNamespace(behavior_id=23))

    def test_sync_callback_mismatch_requests_structure_once_for_running_onboard_behavior(self):
        """Sync mismatch should stop the current mirror and request structure exactly once."""
        launched = []
        published = []
        notifications = []
        mirror = self._make_mirror()
        mirror._active_id = 23
        mirror._sm = types.SimpleNamespace(id=23)
        mirror._running = True
        mirror._starting = False
        mirror._last_obe_status = BEStatus.RUNNING
        mirror._request_struct_pub = types.SimpleNamespace(publish=lambda msg: published.append(msg.data))
        mirror._notify_state_change = lambda: notifications.append(True)
        mirror._log_exception = self._noop

        class _Thread:

            def __init__(self, target, args):
                self.target = target
                self.args = args
                self.daemon = False

            def start(self):
                launched.append((self.target, self.args, self.daemon))
                mirror._running = False

        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _Thread), \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               logwarn=self._noop,
                               localwarn=self._noop,
                               localinfo=self._noop):
            mirror._sync_callback(types.SimpleNamespace(behavior_id=99))
            mirror._sync_callback(types.SimpleNamespace(behavior_id=99))

        self.assertEqual(1, len(launched))
        self.assertTrue(launched[0][2])
        self.assertEqual([99], published)
        self.assertEqual([True], notifications)
        self.assertTrue(mirror._starting)

    def test_sync_callback_mismatch_skips_duplicate_structure_request_while_already_starting(self):
        """Sync mismatch should not request structure again once a re-request is already in progress."""
        mirror = self._make_mirror()
        mirror._active_id = 23
        mirror._sm = types.SimpleNamespace(id=23)
        mirror._running = False
        mirror._starting = True
        mirror._last_obe_status = BEStatus.STARTED
        mirror._request_struct_pub = types.SimpleNamespace(
            publish=lambda _msg: (_ for _ in ()).throw(AssertionError('must not republish structure request'))
        )
        mirror._notify_state_change = lambda: (_ for _ in ()).throw(AssertionError('must not notify again'))
        mirror._log_exception = self._noop

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            logwarn=self._noop,
                            localwarn=self._noop,
                            localinfo=self._noop):
            mirror._sync_callback(types.SimpleNamespace(behavior_id=99))

        self.assertTrue(mirror._starting)

    def test_start_mirror_requests_structure_when_missing(self):
        """Start should request structure and leave the mirror in starting mode."""
        published = []
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = False
        mirror._stopping = False
        mirror._active_id = BehaviorSync.INVALID
        mirror._sm = None
        mirror._struct_buffer = []
        mirror._request_struct_pub = types.SimpleNamespace(
            publish=lambda msg: published.append(msg.data)
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            mirror._start_mirror(
                types.SimpleNamespace(behavior_id=41, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([41], published)
        self.assertTrue(mirror._starting)
        self.assertIsNone(mirror._sm)

    def test_start_mirror_handles_wait_stopping_sync_error(self):
        """Start should log wait-stopping SyncError and return without executing."""
        logged = []
        mirror = self._make_mirror()
        mirror._wait_stopping = lambda *_args, **_kwargs: (_ for _ in ()).throw(SyncError('timeout'))
        mirror._log_exception = lambda *args, **kwargs: logged.append((args, kwargs))
        mirror._execute_mirror = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('must not execute after wait_stopping failure')
        )

        mirror._start_mirror(
            types.SimpleNamespace(behavior_id=41, args=[]),
            types.SimpleNamespace(nanoseconds=1)
        )

        self.assertEqual(1, len(logged))
        self.assertEqual('Start wait for mirror stop failed', logged[0][0][0])

    def test_stop_mirror_clears_active_state(self):
        """Stop should clear active mirror bookkeeping after destroying the SM."""
        destroyed = []
        removed = []
        updates = []
        mirror = self._make_mirror()
        mirror._active_id = 17
        mirror._running = True
        mirror._starting = True
        mirror._stopping = False
        mirror._current_struct = object()
        mirror._wait_stop_running = lambda _start_time: setattr(mirror, '_running', False)
        mirror._sm = types.SimpleNamespace(destroy=lambda: destroyed.append(True))
        mirror._outcome_sub = types.SimpleNamespace(
            remove_last_msg=lambda topic, clear_buffer=False: removed.append((topic, clear_buffer))
        )
        mirror._beh_update_pub = types.SimpleNamespace(
            publish=lambda topic, msg: updates.append((topic, msg.data))
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            loginfo=self._noop,
                            logwarn=self._noop):
            mirror._stop_mirror(
                types.SimpleNamespace(behavior_id=17, code=BEStatus.ERROR, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([True], destroyed)
        self.assertEqual([], removed)
        self.assertEqual([(Topics._BEHAVIOR_UPDATE_TOPIC, -1)], updates)
        self.assertEqual(BehaviorSync.INVALID, mirror._active_id)
        self.assertIsNone(mirror._sm)
        self.assertIsNone(mirror._current_struct)
        self.assertFalse(mirror._running)
        self.assertFalse(mirror._starting)
        self.assertFalse(mirror._stopping)

    def test_handle_execution_exception_clears_warning_without_republishing_running(self):
        """Execution exceptions should clear sync-warning state without emitting a false RUNNING."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._running = True
        mirror._mirror_sync_warning_active = True
        mirror._mirror_status_pub = publisher
        mirror._log_exception = self._noop
        notifications = []
        mirror._notify_state_change = lambda: notifications.append(True)

        FlexbeMirror._handle_execution_exception(
            mirror,
            'Exception in start_mirror',
            RuntimeError('boom'),
            types.SimpleNamespace(nanoseconds=1)
        )

        self.assertFalse(mirror._running)
        self.assertFalse(mirror._mirror_sync_warning_active)
        self.assertEqual([], statuses)
        self.assertEqual([True], notifications)

    def test_sync_warning_recovery_does_not_publish_running_while_stopping(self):
        """Sync recovery should not emit RUNNING once mirror teardown has started."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._mirror_status_pub = publisher
        mirror._running = False
        mirror._stopping = True
        mirror._mirror_sync_warning_active = True

        FlexbeMirror._set_sync_warning_state(mirror, False, behavior_id=41)

        self.assertFalse(mirror._mirror_sync_warning_active)
        self.assertEqual([], statuses)

    def test_mirror_transition_callback_publishes_running_status(self):
        """Mirror transitions should publish RUNNING on the dedicated mirror status topic."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._mirror_status_pub = publisher
        mirror._active_id = 17
        mirror._running = True

        mirror._mirror_transition_callback([types.SimpleNamespace(state_id=1)])

        self.assertEqual([{'code': BEStatus.RUNNING, 'behavior_id': 17, 'args': []}], statuses)

    def test_mirror_status_sequence_covers_lifecycle_warning_and_recovery(self):
        """Mirror status stream should follow the expected startup, run, warning, recovery, and stop sequence."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._mirror_status_pub = publisher
        mirror._beh_update_pub = types.SimpleNamespace(publish=self._noop)
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1))
        mirror._struct_buffer = deque()
        mirror._running = False
        mirror._starting = True
        mirror._start_requested = True
        mirror._active_id = BehaviorSync.INVALID
        mirror._pending_start_args = ['/root/branch']
        mirror._state_map = types.SimpleNamespace(
            get_state=lambda _state_id: types.SimpleNamespace(path='/root/branch')
        )

        built_sm = types.SimpleNamespace(id=41, set_name=self._noop)

        def build(_struct):
            mirror._sm = built_sm

        mirror._mirror_state_machine = build
        mirror._execute_mirror = self._noop

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localinfo_throttle=self._noop,
                            localwarn=self._noop,
                            localwarn_throttle=self._noop,
                            localerr=self._noop,
                            loginfo=self._noop,
                            logwarn=self._noop,
                            warning=self._noop,
                            error=self._noop,
                            info=self._noop):
            mirror._publish_mirror_status(BEStatus.READY)

            FlexbeMirror._activate_mirror(
                mirror,
                types.SimpleNamespace(behavior_id=41),
                types.SimpleNamespace(nanoseconds=1)
            )

            mirror._mirror_transition_callback([types.SimpleNamespace(state_id=1)])

            mirror._sm.get_latest_status = lambda: types.SimpleNamespace(
                behavior_id=41,
                current_state_checksums=[(5 << 8)]
            )

            with patch('flexbe_mirror.flexbe_mirror.StateMap.unhash', return_value=(5, 0)):
                mismatch_msg = types.SimpleNamespace(behavior_id=41, current_state_checksums=[(6 << 8)])
                mirror._onboard_heartbeat_callback(mismatch_msg)
                mirror._onboard_heartbeat_callback(mismatch_msg)
                mirror._onboard_heartbeat_callback(
                    types.SimpleNamespace(behavior_id=41, current_state_checksums=[(5 << 8)])
                )

            mirror._running = True
            mirror._sm = types.SimpleNamespace(destroy=self._noop)
            mirror._current_struct = object()
            mirror._wait_stop_running = lambda _start_time: setattr(mirror, '_running', False)
            mirror._stop_mirror(
                types.SimpleNamespace(behavior_id=41, code=BEStatus.FINISHED, args=['finished']),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([
            {'code': BEStatus.READY, 'behavior_id': 0, 'args': []},
            {'code': BEStatus.STARTED, 'behavior_id': 41, 'args': ['/root/branch']},
            {'code': BEStatus.RUNNING, 'behavior_id': 41, 'args': []},
            {'code': BEStatus.WARNING, 'behavior_id': 41, 'args': []},
            {'code': BEStatus.RUNNING, 'behavior_id': 41, 'args': []},
            {'code': BEStatus.FINISHED, 'behavior_id': 41, 'args': ['finished']},
            {'code': BEStatus.STOPPED, 'behavior_id': 41, 'args': ['finished']},
            {'code': BEStatus.READY, 'behavior_id': 0, 'args': []},
        ], statuses)

    def test_restart_mirror_reuses_existing_state_machine(self):
        """Restart should reinitialize and execute a matching existing mirror."""
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = True
        mirror._active_id = 9
        mirror._starting_path = 'stale'
        mirror._state_map = {}
        mirror._current_struct = types.SimpleNamespace(behavior_id=9)
        mirror._outcome_sub = types.SimpleNamespace(
            remove_last_msg=self._noop
        )
        mirror._request_struct_pub = types.SimpleNamespace(publish=self._noop)
        calls = []
        active_state = types.SimpleNamespace(state_id=55, name='leaf', path='/root/leaf')
        sm = types.SimpleNamespace(
            id=9,
            _states=[],
            _current_state=active_state,
            _last_deep_states_list=['stale'],
            get_deep_states=lambda: [active_state],
            get_latest_status=lambda: 'status'
        )
        mirror._sm = sm
        mirror._reinitialize_state_machine = lambda state_machine: calls.append(('reinit', state_machine))
        mirror._execute_mirror = lambda _start_time: calls.append(('execute', mirror._active_id))

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            loginfo=self._noop,
                            logwarn=self._noop):
            with patch('flexbe_mirror.flexbe_mirror.MirrorState.publish_update', self._noop):
                mirror._restart_mirror(
                    types.SimpleNamespace(behavior_id=9, current_state_checksums=[]),
                    types.SimpleNamespace(nanoseconds=1)
                )

        self.assertEqual([('reinit', sm), ('execute', 9)], calls)
        self.assertEqual(9, mirror._active_id)
        self.assertTrue(mirror._running)
        self.assertFalse(mirror._starting)
        self.assertIsNone(mirror._starting_path)

    def test_start_mirror_uses_matching_buffered_structure(self):
        """Start should select the matching buffered structure and discard stale ones."""
        published = []
        built = []
        executed = []
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = False
        mirror._stopping = False
        mirror._active_id = BehaviorSync.INVALID
        mirror._sm = None
        mirror._state_map = {101: types.SimpleNamespace(path='/root/child')}
        mirror._current_struct = types.SimpleNamespace(containers=[types.SimpleNamespace(state_id=101)])
        mirror._request_struct_pub = types.SimpleNamespace(
            publish=lambda msg: published.append(msg.data)
        )
        mirror._struct_buffer = deque([
            types.SimpleNamespace(behavior_id=7),
            types.SimpleNamespace(behavior_id=41)
        ])

        def build(struct):
            built.append(struct.behavior_id)
            if struct.behavior_id == 41:
                mirror._sm = types.SimpleNamespace(id=41, _current_state=None)

        mirror._mirror_state_machine = build
        mirror._execute_mirror = lambda _start_time: executed.append(mirror._active_id)

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            loginfo=self._noop,
                            logwarn=self._noop):
            mirror._start_mirror(
                types.SimpleNamespace(behavior_id=41, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([41], built)
        self.assertEqual([], published)
        self.assertEqual([41], executed)
        self.assertEqual(41, mirror._active_id)
        self.assertFalse(mirror._starting)

    def test_start_mirror_leaves_fresh_structure_uninitialized_until_spin(self):
        """STARTED should not seed the mirror current state from the structure root entry."""
        executed = []
        fresh_sm = types.SimpleNamespace(id=41, _current_state=None)
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = False
        mirror._stopping = False
        mirror._active_id = BehaviorSync.INVALID
        mirror._sm = fresh_sm
        mirror._current_struct = types.SimpleNamespace(
            containers=[
                types.SimpleNamespace(state_id=1),
                types.SimpleNamespace(state_id=101)
            ]
        )
        mirror._state_map = {
            1: types.SimpleNamespace(path=''),
            101: types.SimpleNamespace(path='/root/child')
        }
        mirror._struct_buffer = deque()
        mirror._outcome_sub = types.SimpleNamespace(remove_last_msg=self._noop)
        mirror._execute_mirror = lambda _start_time: executed.append(mirror._sm._current_state)

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            mirror._start_mirror(
                types.SimpleNamespace(behavior_id=41, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([None], executed)
        self.assertIsNone(fresh_sm._current_state)
        self.assertEqual(41, mirror._active_id)
        self.assertTrue(mirror._running)

    def test_activate_mirror_preloads_structure_without_start_request(self):
        """Structure arrival alone should preload the mirror but not execute it."""
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = False
        mirror._stopping = False
        mirror._active_id = BehaviorSync.INVALID
        mirror._struct_buffer = deque()
        mirror._sm = None
        mirror._current_struct = None

        built_sm = types.SimpleNamespace(id=41, set_name=self._noop)

        def build(_struct):
            mirror._sm = built_sm

        mirror._mirror_state_machine = build
        mirror._execute_mirror = lambda _start_time: (_ for _ in ()).throw(
            AssertionError('structure preload should not execute without STARTED')
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            mirror._activate_mirror(
                types.SimpleNamespace(behavior_id=41),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertIs(mirror._sm, built_sm)
        self.assertEqual(BehaviorSync.INVALID, mirror._active_id)
        self.assertFalse(mirror._running)

    def test_activate_mirror_preserves_deferred_started_args(self):
        """Deferred activation should republish STARTED with the original onboard args."""
        executed = []
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = True
        mirror._stopping = False
        mirror._start_requested = True
        mirror._active_id = BehaviorSync.INVALID
        mirror._pending_start_args = ['/root/branch']
        mirror._mirror_status_pub = publisher
        mirror._struct_buffer = deque()

        built_sm = types.SimpleNamespace(id=41, set_name=self._noop)

        def build(_struct):
            mirror._sm = built_sm

        mirror._mirror_state_machine = build
        mirror._execute_mirror = lambda _start_time: executed.append(mirror._active_id)

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            FlexbeMirror._activate_mirror(
                mirror,
                types.SimpleNamespace(behavior_id=41),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([41], executed)
        self.assertEqual(41, mirror._active_id)
        self.assertTrue(mirror._running)
        self.assertEqual(
            [{'code': BEStatus.STARTED, 'behavior_id': 41, 'args': ['/root/branch']}],
            statuses
        )

    def test_activate_mirror_handles_wait_stopping_sync_error(self):
        """Activation should log wait-stopping SyncError and return without executing."""
        logged = []
        mirror = self._make_mirror()
        mirror._wait_stopping = lambda *_args, **_kwargs: (_ for _ in ()).throw(SyncError('timeout'))
        mirror._log_exception = lambda *args, **kwargs: logged.append((args, kwargs))
        mirror._execute_mirror = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('must not execute after wait_stopping failure')
        )

        FlexbeMirror._activate_mirror(
            mirror,
            types.SimpleNamespace(behavior_id=41),
            types.SimpleNamespace(nanoseconds=1)
        )

        self.assertEqual(1, len(logged))
        self.assertEqual('Activation wait for mirror stop failed', logged[0][0][0])

    def test_start_mirror_overlap_clears_stale_outcomes_before_restart(self):
        """Forced restart should clear cached outcomes before the replacement run starts."""
        removed = []
        destroyed = []
        executed = []
        mirror = self._make_mirror()
        mirror._running = True
        mirror._starting = False
        mirror._stopping = False
        mirror._active_id = 9
        mirror._sm = types.SimpleNamespace(id=9, destroy=lambda: destroyed.append(True))
        mirror._current_struct = types.SimpleNamespace(behavior_id=9)
        mirror._state_map = {101: types.SimpleNamespace(path='/root/child')}
        mirror._struct_buffer = deque([types.SimpleNamespace(behavior_id=41)])
        mirror._wait_stop_running = lambda _start_time: setattr(mirror, '_running', False)

        def build(struct):
            mirror._sm = types.SimpleNamespace(id=struct.behavior_id, _current_state=None)

        mirror._mirror_state_machine = build
        mirror._execute_mirror = lambda _start_time: executed.append(mirror._active_id)
        mirror._outcome_sub = types.SimpleNamespace(
            remove_last_msg=lambda topic, clear_buffer=False: removed.append((topic, clear_buffer)),
            has_buffered=lambda _topic: False,
            get_from_buffer=lambda _topic: (_ for _ in ()).throw(
                AssertionError('buffer should already be clear')
            )
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            loginfo=self._noop,
                            logwarn=self._noop):
            mirror._start_mirror(
                types.SimpleNamespace(behavior_id=41, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([True], destroyed)
        self.assertEqual([(Topics._OUTCOME_TOPIC, True)], removed)
        self.assertEqual([41], executed)
        self.assertEqual(41, mirror._active_id)
        self.assertTrue(mirror._running)

    def test_status_callback_defers_started_during_soft_stop(self):
        """A new STARTED should wait for the current graceful stop to finish."""
        mirror = self._make_mirror()
        mirror._running = True
        mirror._active_id = 17
        mirror._soft_stop_requested = True
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1))
        notifications = []
        mirror._notify_state_change = lambda: notifications.append(True)

        class _UnexpectedThread:

            def __init__(self, *args, **kwargs):
                raise AssertionError('deferred start should not spawn a thread yet')

        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _UnexpectedThread), \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localwarn=self._noop,
                               logwarn=self._noop,
                               logerr=self._noop):
            mirror._status_callback(
                types.SimpleNamespace(code=BEStatus.STARTED, behavior_id=41, args=['/root/branch'])
            )

        self.assertTrue(mirror._start_requested)
        self.assertTrue(mirror._starting)
        self.assertTrue(mirror._soft_stop_requested)
        self.assertEqual(41, mirror._pending_start_behavior_id)
        self.assertEqual(['/root/branch'], mirror._pending_start_args)
        self.assertEqual([True], notifications)

    def test_status_callback_preserves_deferred_started_on_followup_ready(self):
        """A stale READY for the stopping behavior must not erase a queued replacement STARTED."""
        mirror = self._make_mirror()
        mirror._running = True
        mirror._active_id = 17
        mirror._sm = types.SimpleNamespace(id=17)
        mirror._soft_stop_requested = True
        mirror._start_requested = True
        mirror._starting = True
        mirror._pending_start_behavior_id = 41
        mirror._pending_start_args = ['/root/branch']
        mirror._soft_stop_thread = types.SimpleNamespace(is_alive=lambda: True)
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1))

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop,
                            logerr=self._noop):
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.READY, behavior_id=17, args=[]))

        self.assertTrue(mirror._start_requested)
        self.assertTrue(mirror._starting)
        self.assertTrue(mirror._soft_stop_requested)
        self.assertEqual(41, mirror._pending_start_behavior_id)
        self.assertEqual(['/root/branch'], mirror._pending_start_args)

    def test_status_callback_clears_start_request_on_idle_ready_and_logs_warning_status(self):
        """Idle READY should clear pending starts, while WARNING should log without mutating idle state."""
        mirror = self._make_mirror()
        mirror._start_requested = True
        mirror._sm = None
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1))
        mirror._last_stop_status_code = None
        mirror._last_stop_behavior_id = BehaviorSync.INVALID

        with patch('flexbe_mirror.flexbe_mirror.Logger.logwarn') as logwarn, \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localwarn=self._noop,
                               logerr=self._noop):
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.READY, behavior_id=0, args=[]))
            mirror._sm = types.SimpleNamespace(id=41)
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.WARNING, behavior_id=41, args=[]))

        self.assertFalse(mirror._start_requested)
        logwarn.assert_called_once()

    def test_status_callback_records_terminal_status_and_soft_stops_once_on_ready(self):
        """Terminal statuses should be recorded until READY or STOPPED launches one graceful stop."""
        launched = []
        mirror = self._make_mirror()
        mirror._sm = types.SimpleNamespace(id=41)
        mirror._active_id = 41
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1))
        mirror._soft_stop_thread = None
        mirror._last_stop_behavior_id = BehaviorSync.INVALID
        mirror._last_stop_status_code = None

        class _Thread:

            def __init__(self, target, args, name):
                self.target = target
                self.args = args
                self.name = name
                self.daemon = False
                self._alive = False

            def start(self):
                launched.append((self.target, self.args, self.name, self.daemon))
                self._alive = True

            def is_alive(self):
                return self._alive

        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _Thread), \
                patch('flexbe_mirror.flexbe_mirror.Logger.logerr') as logerr, \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localwarn=self._noop,
                               logwarn=self._noop):
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.ERROR, behavior_id=41, args=[]))
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.FINISHED, behavior_id=41, args=[]))
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.READY, behavior_id=41, args=[]))

        self.assertEqual(1, len(launched))
        self.assertIn('soft_stop_41_1', launched[0][2])
        self.assertTrue(launched[0][3])
        self.assertTrue(mirror._soft_stop_requested)
        self.assertEqual(BEStatus.FINISHED, mirror._pending_terminal_status_code)
        self.assertEqual(41, mirror._pending_terminal_status_behavior_id)
        self.assertEqual([], mirror._pending_terminal_status_args)
        logerr.assert_called_once()

    def test_status_callback_logs_normal_active_and_idle_non_ready_paths(self):
        """Status callback should stay passive for normal active and idle non-READY updates."""
        mirror = self._make_mirror()
        mirror._sm = types.SimpleNamespace(id=41)
        mirror._start_requested = True
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1))

        class _UnexpectedThread:

            def __init__(self, *args, **kwargs):
                raise AssertionError('normal or idle status handling should not spawn threads')

        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _UnexpectedThread), \
                patch('flexbe_mirror.flexbe_mirror.Logger.localinfo') as localinfo, \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localwarn=self._noop,
                               logwarn=self._noop,
                               logerr=self._noop):
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.RUNNING, behavior_id=41, args=[]))
            mirror._sm = None
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.ERROR, behavior_id=0, args=[]))

        self.assertEqual(BEStatus.ERROR, mirror._last_obe_status)
        self.assertTrue(mirror._start_requested)
        self.assertFalse(mirror._soft_stop_requested)
        self.assertFalse(mirror._starting)
        self.assertEqual(
            ['Mirror - received BEStatus=%s (%s) normal active mode (%s, %s, %s)',
             'Mirror - received BEStatus=%s (%s) - no active SM (%s, %s, %s)'],
            [call.args[0] for call in localinfo.call_args_list]
        )

    def test_should_ignore_duplicate_ready_stop_matches_stopped_behavior(self):
        """Duplicate READY should only be ignored for the last STOPPED behavior."""
        mirror = self._make_mirror()
        mirror._last_stop_status_code = BEStatus.STOPPED
        mirror._last_stop_behavior_id = 41

        self.assertTrue(mirror._should_ignore_duplicate_ready_stop(
            types.SimpleNamespace(code=BEStatus.READY, behavior_id=41)
        ))
        self.assertTrue(mirror._should_ignore_duplicate_ready_stop(
            types.SimpleNamespace(code=BEStatus.READY, behavior_id=BehaviorSync.INVALID)
        ))
        self.assertFalse(mirror._should_ignore_duplicate_ready_stop(
            types.SimpleNamespace(code=BEStatus.READY, behavior_id=99)
        ))
        self.assertFalse(mirror._should_ignore_duplicate_ready_stop(
            types.SimpleNamespace(code=BEStatus.RUNNING, behavior_id=41)
        ))

    def test_status_callback_launches_start_thread_for_started_and_ignores_duplicate_ready(self):
        """STARTED should launch a start worker, while duplicate READY-after-STOPPED should be ignored."""
        launched = []
        mirror = self._make_mirror()
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=1))
        mirror._last_stop_status_code = BEStatus.STOPPED
        mirror._last_stop_behavior_id = 41

        class _Thread:

            def __init__(self, target, args, name):
                self.target = target
                self.args = args
                self.name = name
                self.daemon = False

            def start(self):
                launched.append((self.target, self.args, self.name, self.daemon))

        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _Thread), \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localwarn=self._noop,
                               logwarn=self._noop,
                               logerr=self._noop):
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.READY, behavior_id=41, args=[]))
            mirror._status_callback(types.SimpleNamespace(code=BEStatus.STARTED, behavior_id=41, args=['/root']))

        self.assertEqual(BEStatus.STARTED, mirror._last_obe_status)
        self.assertTrue(mirror._start_requested)
        self.assertFalse(mirror._soft_stop_requested)
        self.assertEqual(41, mirror._pending_start_behavior_id)
        self.assertEqual(['/root'], mirror._pending_start_args)
        self.assertEqual(1, len(launched))
        self.assertIn('start_mirror_41_1', launched[0][2])
        self.assertTrue(launched[0][3])

    def test_stop_mirror_launches_deferred_start_after_ready(self):
        """Queued STARTED should be launched only after stop publishes READY."""
        statuses, publisher = self._capture_statuses()
        launched = []
        mirror = self._make_mirror()
        mirror._mirror_status_pub = publisher
        mirror._active_id = 17
        mirror._running = True
        mirror._stopping = True
        mirror._sm = types.SimpleNamespace(destroy=self._noop)
        mirror._current_struct = object()
        mirror._beh_update_pub = types.SimpleNamespace(publish=self._noop)
        mirror._start_requested = True
        mirror._starting = True
        mirror._soft_stop_requested = True
        mirror._last_obe_status = BEStatus.READY
        mirror._pending_start_behavior_id = 41
        mirror._pending_start_args = ['/root/branch']
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=99))
        mirror._wait_stop_running = lambda _start_time: (
            setattr(mirror, '_running', False),
            setattr(mirror, '_start_requested', False),
            setattr(mirror, '_starting', False)
        )

        class _DeferredStartThread:

            def __init__(self, target=None, args=None, name=None):
                self.target = target
                self.args = args
                self.name = name
                self.daemon = False

            def start(self):
                launched.append({
                    'target': self.target,
                    'behavior_id': self.args[0].behavior_id,
                    'args': list(self.args[0].args),
                    'name': self.name,
                })

        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _DeferredStartThread), \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localwarn=self._noop,
                               loginfo=self._noop,
                               logwarn=self._noop):
            mirror._stop_mirror(
                types.SimpleNamespace(behavior_id=17, code=BEStatus.FINISHED, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([
            {'code': BEStatus.FINISHED, 'behavior_id': 17, 'args': []},
            {'code': BEStatus.STOPPED, 'behavior_id': 17, 'args': []},
            {'code': BEStatus.READY, 'behavior_id': 0, 'args': []},
        ], statuses)
        self.assertEqual(1, len(launched))
        self.assertEqual('_start_mirror', launched[0]['target'].__name__)
        self.assertEqual(41, launched[0]['behavior_id'])
        self.assertEqual(['/root/branch'], launched[0]['args'])
        self.assertTrue(mirror._start_requested)
        self.assertTrue(mirror._starting)
        self.assertFalse(mirror._stopping)

    def test_soft_stop_watchdog_stops_immediately_when_quiescent(self):
        """Soft-stop should not wait for stall timeout once the mirror is already drained."""
        mirror = self._make_mirror()
        mirror._running = True
        mirror._soft_stop_requested = True
        mirror._wait_poll_sec = 0.01
        mirror._soft_stop_poll_sec = 0.01
        mirror._soft_stop_stall_sec = 10.0
        mirror._soft_stop_timeout_sec = 0.0
        mirror._sm = types.SimpleNamespace(
            _total_loop_count=25,
            _pending_outcomes=deque(),
            _current_state=None
        )
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=7))
        stopped = []
        mirror._stop_mirror = lambda msg, stop_time: stopped.append((msg.behavior_id, stop_time.nanoseconds))

        class _UnexpectedEvent:

            def clear(self):
                return None

            def wait(self, timeout=None):
                raise AssertionError('quiescent soft-stop should not wait for stall timeout')

        mirror._timing_event = _UnexpectedEvent()

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            mirror._soft_stop_watchdog(
                types.SimpleNamespace(behavior_id=41, code=BEStatus.FINISHED, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([(41, 7)], stopped)

    def test_soft_stop_watchdog_forces_stop_when_progress_stalls(self):
        """Soft-stop should force a stop once progress has stalled beyond the configured threshold."""
        mirror = self._make_mirror()
        mirror._running = True
        mirror._soft_stop_requested = True
        mirror._soft_stop_poll_sec = 0.01
        mirror._soft_stop_stall_sec = 0.0
        mirror._soft_stop_timeout_sec = 0.0
        mirror._sm = types.SimpleNamespace(
            _total_loop_count=25,
            _pending_outcomes=deque([1]),
            _current_state=object()
        )
        mirror._system_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=9))
        stopped = []
        mirror._stop_mirror = lambda msg, stop_time: stopped.append((msg.behavior_id, stop_time.nanoseconds))

        class _UnexpectedEvent:

            def clear(self):
                return None

            def wait(self, timeout=None):
                raise AssertionError('stalled soft-stop should force stop before waiting')

        mirror._timing_event = _UnexpectedEvent()

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            mirror._soft_stop_watchdog(
                types.SimpleNamespace(behavior_id=52, code=BEStatus.FINISHED, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([(52, 9)], stopped)

    def test_start_mirror_reuses_matching_preloaded_structure(self):
        """STARTED should reuse a same-id preloaded structure rather than asserting."""
        executed = []
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = False
        mirror._stopping = False
        mirror._active_id = BehaviorSync.INVALID
        mirror._mirror_status_pub = publisher
        mirror._sm = types.SimpleNamespace(id=41, _current_state=types.SimpleNamespace(path='/root/child'))
        mirror._state_map = {101: types.SimpleNamespace(path='/root/child')}
        mirror._current_struct = types.SimpleNamespace(containers=[types.SimpleNamespace(state_id=101)])
        mirror._struct_buffer = deque()
        mirror._outcome_sub = types.SimpleNamespace(remove_last_msg=self._noop)
        mirror._execute_mirror = lambda _start_time: executed.append(mirror._active_id)

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            mirror._start_mirror(
                types.SimpleNamespace(behavior_id=41, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([41], executed)
        self.assertEqual(41, mirror._active_id)
        self.assertTrue(mirror._running)
        self.assertEqual([{'code': BEStatus.STARTED, 'behavior_id': 41, 'args': []}], statuses)

    def test_start_mirror_discards_stale_preloaded_mirror_before_rebuilding(self):
        """STARTED should discard a stale preloaded mirror and rebuild from buffered structure."""
        rebuilt = []
        executed = []
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._mirror_status_pub = publisher
        mirror._sm = types.SimpleNamespace(id=5, destroy=self._noop)
        mirror._current_struct = types.SimpleNamespace(behavior_id=5)
        mirror._outcome_sub = types.SimpleNamespace(remove_last_msg=self._noop)
        mirror._struct_buffer = deque([types.SimpleNamespace(behavior_id=12)])
        mirror._state_map = {}

        def build(struct):
            rebuilt.append(struct.behavior_id)
            mirror._sm = types.SimpleNamespace(id=struct.behavior_id, _current_state=None)

        mirror._mirror_state_machine = build
        mirror._execute_mirror = lambda _start_time: executed.append(mirror._active_id)

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logwarn=self._noop):
            mirror._start_mirror(
                types.SimpleNamespace(behavior_id=12, args=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([12], rebuilt)
        self.assertEqual([12], executed)
        self.assertEqual(12, mirror._active_id)
        self.assertTrue(mirror._running)
        self.assertEqual([{'code': BEStatus.STARTED, 'behavior_id': 12, 'args': []}], statuses)

    def test_restart_mirror_rebuilds_from_current_structure(self):
        """Restart should rebuild from cached structure when the current SM id mismatches."""
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = False
        mirror._active_id = 5
        mirror._state_map = {}
        mirror._current_struct = types.SimpleNamespace(behavior_id=12)
        mirror._outcome_sub = types.SimpleNamespace(remove_last_msg=self._noop)
        mirror._request_struct_pub = types.SimpleNamespace(publish=self._noop)
        rebuilt = []
        executed = []
        active_state = types.SimpleNamespace(state_id=77, name='leaf', path='/root/leaf')
        mirror._sm = types.SimpleNamespace(id=5)

        def build(struct):
            rebuilt.append(struct.behavior_id)
            mirror._sm = types.SimpleNamespace(
                id=12,
                _states=[],
                _current_state=active_state,
                _last_deep_states_list=None,
                get_deep_states=lambda: [active_state],
                get_latest_status=lambda: 'status'
            )

        mirror._mirror_state_machine = build
        mirror._execute_mirror = lambda _start_time: executed.append(mirror._active_id)

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            loginfo=self._noop,
                            logwarn=self._noop):
            with patch('flexbe_mirror.flexbe_mirror.MirrorState.publish_update', self._noop):
                mirror._restart_mirror(
                    types.SimpleNamespace(behavior_id=12, current_state_checksums=[]),
                    types.SimpleNamespace(nanoseconds=1)
                )

        self.assertEqual([12], rebuilt)
        self.assertEqual([12], executed)
        self.assertEqual(12, mirror._active_id)
        self.assertTrue(mirror._running)

    def test_restart_mirror_requests_structure_when_cache_missing(self):
        """Restart should request structure instead of swallowing a missing cache."""
        published = []
        mirror = self._make_mirror()
        mirror._running = False
        mirror._starting = False
        mirror._active_id = 5
        mirror._current_struct = None
        mirror._sm = types.SimpleNamespace(id=99)
        mirror._outcome_sub = types.SimpleNamespace(remove_last_msg=self._noop)
        mirror._request_struct_pub = types.SimpleNamespace(
            publish=lambda msg: published.append(msg.data)
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            loginfo=self._noop,
                            logwarn=self._noop):
            mirror._restart_mirror(
                types.SimpleNamespace(behavior_id=12, current_state_checksums=[]),
                types.SimpleNamespace(nanoseconds=1)
            )

        self.assertEqual([12], published)
        self.assertFalse(mirror._running)
        self.assertIsNone(mirror._sm)

    def test_mirror_state_machine_rejects_sibling_prefix_path_match(self):
        """Validation should reject a state path that only matches by substring."""
        mirror = self._make_mirror()
        destroyed = []
        mirror._state_map_pub = types.SimpleNamespace(publish=self._noop)
        bad_state = types.SimpleNamespace(
            state_id=21,
            path='root/foo_bar_mirror/state_mirror'
        )
        mirror._state_map = types.SimpleNamespace(
            get_state=lambda state_id: bad_state if state_id == 21 else None
        )
        mirror._sm = types.SimpleNamespace(
            id=1,
            destroy=lambda: destroyed.append(True)
        )
        mirror._add_node = self._noop
        msg = types.SimpleNamespace(
            behavior_id=1,
            containers=[
                types.SimpleNamespace(path='root', state_id=1),
                types.SimpleNamespace(path='root/foo/state', state_id=21)
            ]
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            logerr=self._noop,
                            localerr=self._noop):
            mirror._mirror_state_machine(msg)

        self.assertEqual([True], destroyed)
        self.assertIsNone(mirror._sm)

    def test_index_structure_requires_a_top_level_container(self):
        """Mirror reconstruction should reject structure messages without a root container path."""
        msg = types.SimpleNamespace(
            containers=[
                types.SimpleNamespace(path='root/child', children=[], state_id=5),
            ]
        )

        with self.assertRaises(KeyError):
            FlexbeMirror._index_structure(msg)

        self.assertEqual('/root_mirror/child_mirror', FlexbeMirror._expected_mirror_path('/root/child'))

    def test_mirror_state_machine_keeps_constructed_sm_when_state_map_publish_fails(self):
        """A state-map publish failure should log locally but preserve the constructed mirror."""
        mirror = self._make_mirror()
        state = types.SimpleNamespace(state_id=21, path='root_mirror/child_mirror')
        sm = types.SimpleNamespace(name='root_mirror', id=None, destroy=self._noop)
        mirror._outcome_sub = object()
        mirror._mirror_transition_callback = self._noop
        mirror._state_map_pub = types.SimpleNamespace(
            publish=lambda _msg: (_ for _ in ()).throw(TypeError('state map boom'))
        )

        def build(_root, _structure_index):
            mirror._state_map = types.SimpleNamespace(
                items=[(21, state.path)],
                get_state=lambda state_id: state if state_id == 21 else None,
            )
            mirror._sm = sm

        mirror._add_node = build
        msg = types.SimpleNamespace(
            behavior_id=17,
            containers=[
                types.SimpleNamespace(path='root', children=['child'], state_id=1),
                types.SimpleNamespace(path='root/child', children=[], state_id=21),
            ]
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localwarn=self._noop,
                            localerr=self._noop,
                            logerr=self._noop):
            mirror._mirror_state_machine(msg)

        self.assertIs(sm, mirror._sm)
        self.assertEqual(17, mirror._sm.id)
        self.assertIs(mirror._outcome_sub, mirror._sm._outcome_sub)
        self.assertIs(mirror._mirror_transition_callback, mirror._sm._status_event_callback)

    def test_mirror_state_machine_logs_error_when_builder_does_not_create_sm(self):
        """Mirror reconstruction should fail softly if the builder leaves the top-level SM unset."""
        mirror = self._make_mirror()
        mirror._sm = None
        mirror._state_map_pub = types.SimpleNamespace(publish=self._noop)
        mirror._add_node = self._noop
        msg = types.SimpleNamespace(
            behavior_id=23,
            containers=[
                types.SimpleNamespace(path='root', children=[], state_id=23),
            ]
        )

        with patch('flexbe_mirror.flexbe_mirror.Logger.logerr') as logerr, \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localwarn=self._noop,
                               localerr=self._noop):
            mirror._mirror_state_machine(msg)

        logerr.assert_called_once()
        self.assertIsNone(mirror._sm)

    def test_add_node_rejects_invalid_leaf_container_type(self):
        """Leaf nodes must use the plain state type."""
        mirror = self._make_mirror()
        mirror._state_map = types.SimpleNamespace(add_state=self._noop)
        structure_index = {
            'containers_by_path': {
                'root/leaf': types.SimpleNamespace(
                    path='root/leaf',
                    transitions=[],
                    outcomes=[],
                    children=[],
                    type=99,
                    state_id=7,
                    autonomy=[],
                )
            },
            'container_names_by_path': {'root/leaf': 'leaf'},
            'child_paths_by_path': {},
        }

        with self.assertRaises(AssertionError):
            FlexbeMirror._add_node(mirror, 'root/leaf', structure_index)

    def test_add_node_builds_leaf_transition_mapping(self):
        """Leaf nodes should be registered in the state map and transition table."""
        mirror = self._make_mirror()
        added = []

        def add_state(path, state):
            state._state_id = 21
            added.append((path, state))

        mirror._state_map = types.SimpleNamespace(add_state=add_state)
        structure_index = {
            'containers_by_path': {
                'root/leaf': types.SimpleNamespace(
                    path='root/leaf',
                    transitions=['next'],
                    outcomes=['done'],
                    children=[],
                    type=0,
                    state_id=21,
                    autonomy=[0],
                )
            },
            'container_names_by_path': {'root/leaf': 'leaf'},
            'child_paths_by_path': {},
        }

        with patch('flexbe_mirror.flexbe_mirror.MirrorStateMachine.add') as add_mock:
            FlexbeMirror._add_node(mirror, 'root/leaf', structure_index)

        self.assertEqual(1, len(added))
        self.assertEqual('root/leaf', added[0][0])
        add_mock.assert_called_once()
        self.assertEqual('leaf_mirror', add_mock.call_args.args[0])
        self.assertEqual({'done': 'next_mirror'}, add_mock.call_args.kwargs['transitions'])

    def test_add_node_assigns_top_level_mirror_when_container_has_no_transitions(self):
        """Root containers without explicit transitions should become the active top-level mirror."""

        class _FakeMirrorStateMachineInstance:

            def __init__(self):
                self._state_id = 11
                self._total_loop_count = None

            @property
            def state_id(self):
                return self._state_id

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, tb):
                return False

        mirror = self._make_mirror()
        created = _FakeMirrorStateMachineInstance()

        def add_state(_path, state):
            state._state_id = 11

        mirror._state_map = types.SimpleNamespace(add_state=add_state)
        mirror._add_node = self._noop
        mirror._sm = None
        structure_index = {
            'containers_by_path': {
                'root': types.SimpleNamespace(
                    path='root',
                    transitions=[],
                    outcomes=['done'],
                    children=['leaf'],
                    type=1,
                    state_id=11,
                    autonomy=[],
                )
            },
            'container_names_by_path': {'root': 'root'},
            'child_paths_by_path': {'root': ['root/leaf']},
        }

        with patch('flexbe_mirror.flexbe_mirror.MirrorStateMachine', return_value=created):
            FlexbeMirror._add_node(mirror, 'root', structure_index)

        self.assertIs(created, mirror._sm)
        self.assertEqual(0, mirror._sm._total_loop_count)

    def test_shutdown_mirror_cleans_up_timer_and_proxies(self):
        """Shutdown should remove the timer and proxy registrations after stopping."""
        actions = []
        mirror = self._make_mirror()
        mirror._active_id = 31
        mirror._running = False
        mirror._stopping = False
        mirror._sm = None
        mirror._current_struct = object()
        mirror._heartbeat_timer = object()
        mirror.destroy_timer = lambda timer: actions.append(('timer', timer))
        mirror._outcome_sub = types.SimpleNamespace(
            unsubscribe_topic=lambda topic, inst_id=None: actions.append(('unsubscribe', topic, inst_id))
        )
        mirror._beh_update_pub = types.SimpleNamespace(
            remove_publisher=lambda topic: actions.append(('publisher', topic))
        )

        with patch('flexbe_mirror.flexbe_mirror.threading.Event') as event_cls:
            event_cls.return_value.wait = self._noop
            self.assertTrue(mirror.shutdown_mirror())

        self.assertEqual(BehaviorSync.INVALID, mirror._active_id)
        self.assertIsNone(mirror._sm)
        self.assertIsNone(mirror._current_struct)
        self.assertIn(('timer', mirror._heartbeat_timer), actions)
        self.assertIn(('unsubscribe', Topics._OUTCOME_TOPIC, id(mirror)), actions)
        self.assertIn(('publisher', Topics._BEHAVIOR_UPDATE_TOPIC), actions)

    def test_shutdown_mirror_returns_false_when_cleanup_raises(self):
        """Shutdown should fail softly when cleanup raises during teardown."""
        mirror = self._make_mirror()
        mirror._active_id = 31
        mirror._running = False
        mirror._stopping = False
        mirror._sm = None
        mirror._heartbeat_timer = object()
        mirror.destroy_timer = lambda _timer: (_ for _ in ()).throw(RuntimeError('timer boom'))

        with patch('builtins.print') as print_mock:
            self.assertFalse(mirror.shutdown_mirror())

        printed = '\n'.join(str(call.args[0]) for call in print_mock.call_args_list if call.args)
        self.assertIn('Exception shutting down behavior mirror', printed)

    def test_mirror_structure_callback_ignores_shutdown_and_starts_worker_thread(self):
        """Mirror structure callback should skip work during shutdown and start a daemon worker otherwise."""
        launched = []
        mirror = self._make_mirror()
        msg = types.SimpleNamespace(behavior_id=99)

        class _Thread:

            def __init__(self, target, args, name):
                self.target = target
                self.args = args
                self.name = name
                self.daemon = False

            def start(self):
                launched.append((self.target, self.args, self.name, self.daemon))

        mirror._shutdown_requested = True
        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _Thread):
            mirror._mirror_structure_callback(msg)
        self.assertEqual([], launched)

        mirror._shutdown_requested = False
        start_time = types.SimpleNamespace(nanoseconds=123456)
        mirror._system_clock = types.SimpleNamespace(now=lambda: start_time)
        with patch('flexbe_mirror.flexbe_mirror.threading.Thread', _Thread), \
                patch('flexbe_mirror.flexbe_mirror.Logger.localinfo', self._noop):
            mirror._mirror_structure_callback(msg)

        self.assertEqual(1, len(launched))
        self.assertEqual([msg, start_time], launched[0][1])
        self.assertIn('activate_mirror_99_123456', launched[0][2])
        self.assertTrue(launched[0][3])

    def test_preempt_callback_logs_for_running_and_idle_mirror(self):
        """Preempt callback should warn differently depending on whether a mirror exists."""
        mirror = self._make_mirror()
        mirror._sm = types.SimpleNamespace(id=1)

        with patch('flexbe_mirror.flexbe_mirror.Logger.logwarn') as logwarn:
            mirror._preempt_callback(object())
            mirror._sm = None
            mirror._preempt_callback(object())

        self.assertEqual(2, logwarn.call_count)

    def test_onboard_heartbeat_mismatch_counter_resets_when_back_in_sync(self):
        """Heartbeat mismatch counter should increment and then reset on recovery."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._active_id = 44
        mirror._stopping = False
        mirror._sync_heartbeat_mismatch_counter = 0
        mirror._mirror_status_pub = publisher
        state = types.SimpleNamespace(name='leaf_mirror', path='/root/leaf_mirror')
        mirror._state_map = types.SimpleNamespace(get_state=lambda _state_id: state)
        checksum = (5 << 8)
        mirror._sm = types.SimpleNamespace(
            get_latest_status=lambda: types.SimpleNamespace(
                behavior_id=44,
                current_state_checksums=[checksum]
            )
        )
        mismatch_msg = types.SimpleNamespace(behavior_id=44, current_state_checksums=[(6 << 8)])
        sync_msg = types.SimpleNamespace(behavior_id=44, current_state_checksums=[checksum])

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localinfo_throttle=self._noop,
                            localwarn=self._noop,
                            localerr=self._noop,
                            logwarn=self._noop,
                            warning=self._noop,
                            error=self._noop,
                            info=self._noop):
            mirror._onboard_heartbeat_callback(mismatch_msg)
            self.assertEqual(1, mirror._sync_heartbeat_mismatch_counter)
            mirror._onboard_heartbeat_callback(sync_msg)

        self.assertEqual(0, mirror._sync_heartbeat_mismatch_counter)
        self.assertEqual([], statuses)

    def test_onboard_heartbeat_uses_compact_mismatch_summary_between_detailed_dumps(self):
        """Persistent mismatch logging should switch to compact summaries between detailed dumps."""
        mirror = self._make_mirror()
        mirror._active_id = 44
        mirror._stopping = False
        mirror._sync_heartbeat_mismatch_counter = 10
        state = types.SimpleNamespace(name='leaf_mirror', path='/root/leaf_mirror')
        mirror._state_map = types.SimpleNamespace(get_state=lambda _state_id: state)
        checksum = (5 << 8)
        mismatch_msg = types.SimpleNamespace(behavior_id=44, current_state_checksums=[(6 << 8)])
        mirror._last_onboard_mismatch_sig = tuple(mismatch_msg.current_state_checksums)
        mirror._last_mirror_mismatch_sig = (checksum,)
        mirror._sm = types.SimpleNamespace(
            get_latest_status=lambda: types.SimpleNamespace(
                behavior_id=44,
                current_state_checksums=[checksum]
            )
        )

        with patch.object(mirror, '_log_compact_sync_summary') as compact_log, \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localinfo_throttle=self._noop,
                               localwarn=self._noop,
                               localerr=self._noop,
                               logwarn=self._noop,
                               warning=self._noop,
                               error=self._noop,
                               info=self._noop):
            mirror._onboard_heartbeat_callback(mismatch_msg)

        compact_log.assert_called_once()

    def test_onboard_heartbeat_logs_compact_mismatch_on_repeated_signature(self):
        """A repeated mismatch signature should trigger the compact mismatch summary once."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._active_id = 44
        mirror._stopping = False
        mirror._sync_heartbeat_mismatch_counter = 1
        mirror._mirror_status_pub = publisher
        state = types.SimpleNamespace(name='leaf_mirror', path='/root/leaf_mirror')
        mirror._state_map = types.SimpleNamespace(get_state=lambda _state_id: state)
        checksum = (5 << 8)
        mismatch_msg = types.SimpleNamespace(behavior_id=44, current_state_checksums=[(6 << 8)])
        mirror._last_onboard_mismatch_sig = tuple(mismatch_msg.current_state_checksums)
        mirror._last_mirror_mismatch_sig = (checksum,)
        mirror._sm = types.SimpleNamespace(
            get_latest_status=lambda: types.SimpleNamespace(
                behavior_id=44,
                current_state_checksums=[checksum]
            )
        )

        with patch.object(mirror, '_log_compact_sync_summary') as compact_log, \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localinfo_throttle=self._noop,
                               localwarn=self._noop,
                               localerr=self._noop,
                               logwarn=self._noop,
                               warning=self._noop,
                               error=self._noop,
                               info=self._noop):
            mirror._onboard_heartbeat_callback(mismatch_msg)

        compact_log.assert_called_once_with(tuple(mismatch_msg.current_state_checksums), (checksum,))
        self.assertEqual(mirror._last_onboard_mismatch_sig, tuple(mismatch_msg.current_state_checksums))
        self.assertEqual(mirror._last_mirror_mismatch_sig, (checksum,))
        self.assertEqual(2, mirror._sync_heartbeat_mismatch_counter)
        self.assertEqual([{'code': BEStatus.WARNING, 'behavior_id': 44, 'args': []}], statuses)

    def test_onboard_heartbeat_resets_confirmation_when_signature_changes(self):
        """A different mismatch signature on the next heartbeat should remain transient."""
        mirror = self._make_mirror()
        mirror._active_id = 44
        mirror._stopping = False
        mirror._sync_heartbeat_mismatch_counter = 1
        state = types.SimpleNamespace(name='leaf_mirror', path='/root/leaf_mirror')
        mirror._state_map = types.SimpleNamespace(get_state=lambda _state_id: state)
        checksum = (5 << 8)
        mirror._last_onboard_mismatch_sig = ((6 << 8),)
        mirror._last_mirror_mismatch_sig = (checksum,)
        mismatch_msg = types.SimpleNamespace(behavior_id=44, current_state_checksums=[(7 << 8)])
        mirror._sm = types.SimpleNamespace(
            get_latest_status=lambda: types.SimpleNamespace(
                behavior_id=44,
                current_state_checksums=[checksum]
            )
        )

        with patch.object(mirror, '_log_compact_sync_summary') as compact_log, \
                patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                               localinfo=self._noop,
                               localinfo_throttle=self._noop,
                               localwarn=self._noop,
                               localerr=self._noop,
                               logwarn=self._noop,
                               warning=self._noop,
                               error=self._noop,
                               info=self._noop):
            mirror._onboard_heartbeat_callback(mismatch_msg)

        compact_log.assert_not_called()
        self.assertEqual(1, mirror._sync_heartbeat_mismatch_counter)
        self.assertEqual(mirror._last_onboard_mismatch_sig, tuple(mismatch_msg.current_state_checksums))
        self.assertEqual(mirror._last_mirror_mismatch_sig, (checksum,))

    def test_onboard_heartbeat_publishes_running_after_warning_clears(self):
        """Returning to matching heartbeats should publish RUNNING after a sync warning."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._active_id = 44
        mirror._running = True
        mirror._stopping = False
        mirror._sync_heartbeat_mismatch_counter = 2
        mirror._mirror_sync_warning_active = True
        mirror._mirror_status_pub = publisher
        checksum = (5 << 8)
        mirror._state_map = types.SimpleNamespace(get_state=lambda _state_id: types.SimpleNamespace(path='/root/leaf_mirror'))
        mirror._sm = types.SimpleNamespace(
            get_latest_status=lambda: types.SimpleNamespace(
                behavior_id=44,
                current_state_checksums=[checksum]
            )
        )

        with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                            localinfo=self._noop,
                            localinfo_throttle=self._noop,
                            localwarn=self._noop,
                            localerr=self._noop,
                            logwarn=self._noop,
                            warning=self._noop,
                            error=self._noop,
                            info=self._noop):
            mirror._onboard_heartbeat_callback(
                types.SimpleNamespace(behavior_id=44, current_state_checksums=[checksum])
            )

        self.assertEqual([{'code': BEStatus.RUNNING, 'behavior_id': 44, 'args': []}], statuses)
        self.assertFalse(mirror._mirror_sync_warning_active)

    def test_onboard_heartbeat_warns_when_matching_id_has_no_active_mirror(self):
        """Heartbeat should warn when ids match but there is no reconstructed mirror graph."""
        mirror = self._make_mirror()
        mirror._active_id = 44
        mirror._sm = None

        with patch('flexbe_mirror.flexbe_mirror.Logger.warning') as warning:
            mirror._onboard_heartbeat_callback(
                types.SimpleNamespace(behavior_id=44,
                                      current_state_checksums=[],
                                      INVALID=BehaviorSync.INVALID)
            )

        warning.assert_called_once()

    def test_onboard_heartbeat_handles_mismatched_behavior_ids_and_stopping_waits(self):
        """Heartbeat should escalate mismatched behavior ids and log while stopping."""
        mirror = self._make_mirror()
        mirror._active_id = 44
        mirror._stopping = False
        mirror._sync_heartbeat_mismatch_counter = 1

        with patch.object(mirror, '_set_sync_warning_state') as set_warning, \
                patch('flexbe_mirror.flexbe_mirror.Logger.error') as error_log:
            mirror._onboard_heartbeat_callback(
                types.SimpleNamespace(behavior_id=55, current_state_checksums=[], INVALID=BehaviorSync.INVALID)
            )

        set_warning.assert_not_called()
        error_log.assert_called_once()
        self.assertEqual(2, mirror._sync_heartbeat_mismatch_counter)

        mirror._active_id = 44
        mirror._stopping = True
        mirror._sync_heartbeat_mismatch_counter = 0
        with patch('flexbe_mirror.flexbe_mirror.Logger.localinfo') as localinfo:
            mirror._onboard_heartbeat_callback(
                types.SimpleNamespace(behavior_id=BehaviorSync.INVALID,
                                      current_state_checksums=[],
                                      INVALID=BehaviorSync.INVALID)
            )

        localinfo.assert_called_once()

    def test_mirror_status_topic_uses_bestatus_type(self):
        """Mirror status topic should be registered with the shared BEStatus message type."""
        self.assertEqual('flexbe/mirror/status', Topics._MIRROR_STATUS_TOPIC)
        self.assertIs(BEStatus, Topics.get_type(Topics._MIRROR_STATUS_TOPIC))

    def test_publish_mirror_status_serializes_args_and_tolerates_clock_failures(self):
        """Mirror status publication should stringify args and still publish if stamping fails."""
        published = []
        mirror = self._make_mirror()
        stamp = object()
        mirror._mirror_status_pub = types.SimpleNamespace(publish=lambda msg: published.append(msg))
        mirror.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: stamp)
        )

        mirror._publish_mirror_status(BEStatus.RUNNING, behavior_id=12, args=[1, 'two'])
        mirror.get_clock = lambda: types.SimpleNamespace(now=lambda: (_ for _ in ()).throw(RuntimeError('clock')))
        mirror._publish_mirror_status(BEStatus.READY)

        self.assertEqual(2, len(published))
        self.assertEqual(BEStatus.RUNNING, published[0].code)
        self.assertEqual(12, published[0].behavior_id)
        self.assertEqual(['1', 'two'], list(published[0].args))
        self.assertEqual(stamp, published[0].stamp)
        self.assertEqual(BEStatus.READY, published[1].code)

    def test_mirror_status_helpers_manage_warning_latches_and_state_changes(self):
        """Mirror helper methods should latch warnings and wake waiters on state changes."""
        statuses, publisher = self._capture_statuses()
        mirror = self._make_mirror()
        mirror._mirror_status_pub = publisher
        mirror._running = True
        mirror._stopping = False

        mirror._notify_state_change()
        self.assertTrue(mirror._timing_event.is_set())
        mirror._timing_event.clear()

        mirror._set_sync_warning_state(True, behavior_id=33)
        mirror._set_sync_warning_state(True, behavior_id=33)
        mirror._set_sync_warning_state(False, behavior_id=33)
        mirror._clear_sync_warning_state()

        self.assertEqual(
            [
                {'code': BEStatus.WARNING, 'behavior_id': 33, 'args': []},
                {'code': BEStatus.RUNNING, 'behavior_id': 33, 'args': []},
            ],
            statuses,
        )
        self.assertFalse(mirror._mirror_sync_warning_active)

    def test_clear_outcome_tracking_and_heartbeat_cover_idle_and_running_paths(self):
        """Mirror helper methods should clear cached outcomes and publish idle or running heartbeats."""
        removed = []
        heartbeats = []
        mirror = self._make_mirror()
        mirror._outcome_sub = types.SimpleNamespace(
            remove_last_msg=lambda topic, clear_buffer=False: removed.append((topic, clear_buffer))
        )
        mirror._heartbeat_pub = types.SimpleNamespace(publish=lambda msg: heartbeats.append(msg.data))
        mirror.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(seconds_nanoseconds=lambda: (0x12345, 0))
        )
        mirror._sm = None

        mirror._clear_outcome_tracking()
        with patch('flexbe_mirror.flexbe_mirror.Logger.check_local_enabled', self._noop):
            mirror.heartbeat_timer_callback()
            mirror._sm = types.SimpleNamespace(_total_loop_count=37)
            mirror._running = True
            mirror.heartbeat_timer_callback()

        self.assertEqual([(Topics._OUTCOME_TOPIC, True)], removed)
        self.assertEqual([-(0x12345 & 0x0000FFFF), 37], heartbeats)

    def test_soft_stop_snapshot_helpers_cover_none_progress_and_quiescence_cases(self):
        """Soft-stop snapshot helpers should classify progress and quiescence consistently."""
        self.assertIsNone(FlexbeMirror._extract_soft_stop_snapshot(None))
        snapshot = FlexbeMirror._extract_soft_stop_snapshot(
            types.SimpleNamespace(_total_loop_count=5, _pending_outcomes=deque([1, 2]), _current_state=object())
        )
        self.assertEqual((5, 2, True), snapshot)
        self.assertTrue(FlexbeMirror._soft_stop_has_progress(None, snapshot))
        self.assertTrue(FlexbeMirror._soft_stop_has_progress(snapshot, None))
        self.assertFalse(FlexbeMirror._soft_stop_has_progress(snapshot, snapshot))
        self.assertFalse(FlexbeMirror._soft_stop_is_quiescent(snapshot))
        self.assertTrue(FlexbeMirror._soft_stop_is_quiescent((9, 0, False)))

    def test_log_exception_uses_severity_specific_channels_and_optional_trace_output(self):
        """Mirror exception logging should choose the correct logger channel and skip empty traces."""
        mirror = self._make_mirror()
        exc = RuntimeError('boom')

        with patch('flexbe_mirror.flexbe_mirror.map_exception_to_bestatus',
                   side_effect=[BEStatus.WARNING, BEStatus.ERROR]), \
                patch('flexbe_mirror.flexbe_mirror.traceback.format_exc',
                      side_effect=['warning trace', 'NoneType: None']), \
                patch('flexbe_mirror.flexbe_mirror.Logger.logwarn') as logwarn, \
                patch('flexbe_mirror.flexbe_mirror.Logger.logerr') as logerr, \
                patch('flexbe_mirror.flexbe_mirror.Logger.localinfo') as localinfo:
            mirror._log_exception('warning path', exc, start_time=object())
            mirror._log_exception('error path', exc)

        logwarn.assert_called_once()
        logerr.assert_called_once()
        localinfo.assert_called_once()
        self.assertIn('warning path elapsed: RuntimeError - boom', logwarn.call_args.args[0])
        self.assertIn('error path: RuntimeError - boom', logerr.call_args.args[0])

    def test_behavior_mirror_main_handles_keyboard_interrupt_and_shutdown(self):
        """Behavior mirror main should cleanly tear down on keyboard interrupt."""
        init_calls = []
        try_shutdown_calls = []
        destroy_calls = []
        spin_once_calls = []
        log_messages = []

        mirror = types.SimpleNamespace(
            get_logger=lambda: types.SimpleNamespace(info=lambda msg: log_messages.append(msg)),
            shutdown_mirror=lambda: True,
            destroy_node=lambda: destroy_calls.append('destroyed'),
        )

        class _Executor:

            def __init__(self):
                self.node = None
                self.removed_nodes = []
                self.shutdown_calls = 0

            def add_node(self, node):
                self.node = node

            def spin(self):
                raise KeyboardInterrupt

            def spin_once(self, timeout_sec=0.0):
                spin_once_calls.append(timeout_sec)

            def remove_node(self, node):
                self.removed_nodes.append(node)

            def shutdown(self):
                self.shutdown_calls += 1

        executor = _Executor()

        with patch.object(behavior_mirror_sm.rclpy, 'init',
                          side_effect=lambda **kwargs: init_calls.append(kwargs)), \
                patch.object(behavior_mirror_sm.rclpy.executors,
                             'SingleThreadedExecutor',
                             return_value=executor), \
                patch.object(behavior_mirror_sm, 'FlexbeMirror', return_value=mirror), \
                patch.object(behavior_mirror_sm, 'shutdown_proxies') as shutdown_proxies, \
                patch.object(behavior_mirror_sm.rclpy, 'try_shutdown',
                             side_effect=lambda: try_shutdown_calls.append(True)), \
                patch('builtins.print') as print_mock:
            behavior_mirror_sm.main(args=['--demo'])

        self.assertEqual([{
            'args': ['--demo'],
            'signal_handler_options': behavior_mirror_sm.rclpy.signals.SignalHandlerOptions.NO,
        }], init_calls)
        self.assertIs(executor.node, mirror)
        self.assertEqual([mirror], executor.removed_nodes)
        self.assertEqual(1, executor.shutdown_calls)
        self.assertEqual(['Begin behavior mirror processing ...'], log_messages)
        self.assertEqual(['destroyed'], destroy_calls)
        self.assertEqual([True], try_shutdown_calls)
        self.assertEqual([], spin_once_calls)
        shutdown_proxies.assert_called_once()

        printed = '\n'.join(str(call.args[0]) for call in print_mock.call_args_list if call.args)
        self.assertIn('Keyboard interrupt', printed)
        self.assertIn('Done with behavior mirror', printed)

    def test_behavior_mirror_main_treats_invalid_handle_as_shutdown(self):
        """Destroy-request InvalidHandle from executor.spin should not be reported as a crash."""
        mirror = types.SimpleNamespace(
            get_logger=lambda: types.SimpleNamespace(info=self._noop),
            shutdown_mirror=lambda: True,
            destroy_node=self._noop,
        )

        class _Executor:

            def __init__(self):
                self.removed_nodes = []
                self.shutdown_calls = 0

            def add_node(self, _node):
                return None

            def spin(self):
                raise behavior_mirror_sm.InvalidHandle('destruction was requested')

            def remove_node(self, node):
                self.removed_nodes.append(node)

            def shutdown(self):
                self.shutdown_calls += 1

        with patch.object(behavior_mirror_sm.rclpy, 'init'), \
                patch.object(behavior_mirror_sm.rclpy.executors,
                             'SingleThreadedExecutor',
                             return_value=_Executor()), \
                patch.object(behavior_mirror_sm, 'FlexbeMirror', return_value=mirror), \
                patch.object(behavior_mirror_sm, 'shutdown_proxies'), \
                patch.object(behavior_mirror_sm.rclpy, 'try_shutdown'), \
                patch('builtins.print') as print_mock:
            behavior_mirror_sm.main()

        printed = '\n'.join(str(call.args[0]) for call in print_mock.call_args_list if call.args)
        self.assertIn('Behavior mirror executor stopped during shutdown', printed)
        self.assertNotIn('Exception in mirror executor!', printed)

    def test_behavior_mirror_main_retries_shutdown_and_logs_exceptions(self):
        """Behavior mirror main should retry shutdown and log executor or proxy teardown failures."""
        init_calls = []
        shutdown_results = deque([False, True])
        spin_once_calls = []

        mirror = types.SimpleNamespace(
            get_logger=lambda: types.SimpleNamespace(info=self._noop),
            shutdown_mirror=lambda: shutdown_results.popleft(),
            destroy_node=self._noop,
        )

        class _Executor:

            def add_node(self, _node):
                return None

            def spin(self):
                raise RuntimeError('spin boom')

            def spin_once(self, timeout_sec=0.0):
                spin_once_calls.append(timeout_sec)

        with patch.object(behavior_mirror_sm.rclpy, 'init',
                          side_effect=lambda **kwargs: init_calls.append(kwargs)), \
                patch.object(behavior_mirror_sm.rclpy.executors,
                             'SingleThreadedExecutor',
                             return_value=_Executor()), \
                patch.object(behavior_mirror_sm, 'FlexbeMirror', return_value=mirror), \
                patch.object(behavior_mirror_sm, 'shutdown_proxies',
                             side_effect=RuntimeError('proxy boom')) as shutdown_proxies, \
                patch.object(behavior_mirror_sm.rclpy, 'try_shutdown',
                             side_effect=RuntimeError('shutdown boom')), \
                patch('builtins.print') as print_mock:
            behavior_mirror_sm.main()

        self.assertEqual([{
            'args': None,
            'signal_handler_options': behavior_mirror_sm.rclpy.signals.SignalHandlerOptions.NO,
        }], init_calls)
        self.assertEqual([0.001] * 100, spin_once_calls)
        shutdown_proxies.assert_called_once()

        printed = '\n'.join(str(call.args[0]) for call in print_mock.call_args_list if call.args)
        self.assertIn('Exception in mirror executor!', printed)
        self.assertIn('Exception in behavior mirror node shutdown!', printed)
        self.assertIn('Exception from rclpy.try_shutdown for behavior mirror', printed)

    def test_execute_mirror_clears_running_when_sync_error_raised(self):
        """_running must be False after _execute_mirror even when SyncError propagates."""
        mirror = self._make_mirror()
        mirror._running = True
        mirror._active_thread_start = None
        mirror._active_id = 42
        mirror._state_map = {}
        mirror._starting_path = None
        PreemptableState.preempt = False

        class _RaisingSM:
            id = 42  # noqa: A003

            def spin(self, *args, **kwargs):
                raise SyncError('test sync error')

        mirror._sm = _RaisingSM()

        try:
            with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                                loginfo=self._noop,
                                localinfo=self._noop,
                                localwarn=self._noop,
                                logerr=self._noop,
                                localerr=self._noop):
                with self.assertRaises(SyncError):
                    mirror._execute_mirror(types.SimpleNamespace(nanoseconds=1))
        finally:
            PreemptableState.preempt = False

        self.assertFalse(mirror._running, '_running must be False after SyncError propagation')

    def test_execute_mirror_clears_running_when_transition_error_raised(self):
        """_running must be False after _execute_mirror even when TransitionError propagates."""
        mirror = self._make_mirror()
        mirror._running = True
        mirror._active_thread_start = None
        mirror._active_id = 42
        mirror._state_map = {}
        mirror._starting_path = None
        PreemptableState.preempt = False

        class _RaisingSM:
            id = 42  # noqa: A003

            def spin(self, *args, **kwargs):
                raise TransitionError('test transition error')

        mirror._sm = _RaisingSM()

        try:
            with patch.multiple('flexbe_mirror.flexbe_mirror.Logger',
                                loginfo=self._noop,
                                localinfo=self._noop,
                                localwarn=self._noop,
                                logerr=self._noop,
                                localerr=self._noop):
                with self.assertRaises(TransitionError):
                    mirror._execute_mirror(types.SimpleNamespace(nanoseconds=1))
        finally:
            PreemptableState.preempt = False

        self.assertFalse(mirror._running, '_running must be False after TransitionError propagation')


if __name__ == '__main__':
    unittest.main()
