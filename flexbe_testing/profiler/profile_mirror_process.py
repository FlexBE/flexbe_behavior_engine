#!/usr/bin/env python3

"""Profile FlexBE mirror in its own process while onboard runs elsewhere."""

import argparse
import cProfile
import os
import sys
import threading
import time
from collections import deque
from io import StringIO
from pstats import Stats
from statistics import pstdev

from flexbe_core.core.topics import Topics

from flexbe_mirror.flexbe_mirror import FlexbeMirror

from flexbe_msgs.msg import BEStatus, BehaviorSync

import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


# Mirror WARNING is a sync-warning signal, not a terminal run outcome.
TERMINAL_STATUSES = (BEStatus.FINISHED, BEStatus.FAILED, BEStatus.ERROR)


def _timing_summary(values):
    """Return min/max/avg/std summary for a non-empty numeric sequence."""
    average = sum(values) / len(values)
    return min(values), max(values), average, pstdev(values)


def _wait_for_match(predicate, timeout_sec, interval_sec=0.05):
    """Wait until a pub/sub match or state predicate becomes true."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_sec)
    return predicate()


def _write_tty_status(message):
    """Write a progress message directly to the controlling terminal if available."""
    try:
        with open('/dev/tty', 'w', encoding='utf-8') as tty:
            print(message, file=tty)
    except OSError:
        pass


def _emit_output_line(line):
    """Write one output line immediately so late cleanup failures do not lose the report."""
    print(line, flush=True)


def _emit_profile_summary(profile, output_path, sort_key, top_count):
    """Write and print a coarse aggregate cProfile summary."""
    stats = Stats(profile)
    stats.dump_stats(output_path)
    _emit_output_line(f'profile_output={output_path}')

    stream = StringIO()
    stats.stream = stream
    stats.strip_dirs().sort_stats(sort_key)
    stats.print_stats(top_count)

    for line in stream.getvalue().splitlines():
        if line.strip():
            _emit_output_line(f'profile: {line}')


class MirrorStatusObserver:
    """Track mirror lifecycle using the dedicated mirror status topic."""

    def __init__(self):
        self.condition = threading.Condition()
        self._event_queue = deque()
        self._pending_event = None
        self.last_status = None

    def status_callback(self, msg):
        """Track mirror status messages."""
        now = time.perf_counter()
        with self.condition:
            self.last_status = msg.code
            self._event_queue.append({
                'code': msg.code,
                'behavior_id': msg.behavior_id,
                'timestamp': now,
            })
            self.condition.notify_all()

    def wait_for_ready(self, timeout_sec):
        """Wait until READY is observed."""
        return self._wait_for(lambda: self.last_status == BEStatus.READY, timeout_sec)

    def collect_next_run(self, timeout_sec):
        """Collect the next complete run from the mirror status stream."""
        state = {
            'status_count': 0,
            'started': False,
            'behavior_id': BehaviorSync.INVALID,
            'started_at': None,
            'terminal_status': None,
            'terminal_time': None,
            'stopped_time': None,
            'ready_time': None,
            'boundary': 'timeout',
            'status_note': 'missing_STARTED',
            'last_code': None,
        }
        deadline = time.monotonic() + timeout_sec

        with self.condition:
            while time.monotonic() < deadline:
                event = None
                if self._pending_event is not None:
                    event = self._pending_event
                    self._pending_event = None
                elif self._event_queue:
                    event = self._event_queue.popleft()

                if event is None:
                    self.condition.wait(timeout=0.05)
                    continue

                code = event['code']
                behavior_id = event['behavior_id']
                timestamp = event['timestamp']
                state['last_code'] = code

                if not state['started']:
                    if code != BEStatus.STARTED:
                        continue
                    state['started'] = True
                    state['behavior_id'] = behavior_id
                    state['started_at'] = timestamp
                    state['status_count'] = 1
                    state['status_note'] = 'missing_terminal'
                    continue

                if state['terminal_time'] is not None and code == BEStatus.STARTED:
                    self._pending_event = event
                    state['boundary'] = 'restarted'
                    return state

                state['status_count'] += 1

                if code in TERMINAL_STATUSES:
                    if state['terminal_time'] is None:
                        state['terminal_status'] = code
                        state['terminal_time'] = timestamp
                        state['status_note'] = 'missing_STOPPED'
                    else:
                        state['status_note'] = f'unexpected_status_{code}'
                    continue

                if code == BEStatus.STOPPED and state['terminal_time'] is not None:
                    if state['stopped_time'] is None:
                        state['stopped_time'] = timestamp
                        state['status_note'] = 'missing_READY'
                    else:
                        state['status_note'] = 'duplicate_STOPPED'
                    continue

                if code == BEStatus.READY and state['stopped_time'] is not None:
                    state['ready_time'] = timestamp
                    state['status_note'] = 'ok'
                    state['boundary'] = 'ready'
                    return state

                if code in (BEStatus.RUNNING, BEStatus.WARNING):
                    if state['terminal_time'] is not None:
                        state['status_note'] = f'unexpected_status_{code}'
                        return state
                    continue

                state['status_note'] = f'unexpected_status_{code}'
                return state

        return state

    def _wait_for(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        with self.condition:
            while time.monotonic() < deadline:
                if predicate():
                    return True
                self.condition.wait(timeout=0.05)
        return predicate()


def _is_mirror_quiescent(mirror):
    """Return whether mirror is fully idle between runs."""
    return (
        not mirror._running
        and not mirror._starting
        and not mirror._stopping
        and mirror._active_id == BehaviorSync.INVALID
        and mirror._sm is None
    )


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=3, help='Number of measured runs to wait for.')
    parser.add_argument('--warmup-runs', type=int, default=1, help='Warmup runs to ignore before measurement.')
    parser.add_argument('--startup-timeout', type=float, default=120.0,
                        help='How long to wait for the external onboard process to appear and for mirror READY.')
    parser.add_argument('--timeout', type=float, default=30.0, help='Timeout per externally driven run in seconds.')
    parser.add_argument('--profile-output', default='',
                        help='Optional aggregate cProfile output file for mirror executor callback work.')
    parser.add_argument('--profile-sort', default='cumulative',
                        help='Sort key for printed aggregate cProfile stats (default: cumulative).')
    parser.add_argument('--profile-top', type=int, default=25,
                        help='How many aggregate cProfile rows to print.')
    return parser.parse_args()


def main():
    """Run the standalone split-process mirror profiling harness."""
    args = _parse_args()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

    rclpy.init()
    executor = SingleThreadedExecutor()
    helper_node = rclpy.create_node('mirror_profile_process_driver')
    observer = MirrorStatusObserver()
    mirror = FlexbeMirror()

    executor.add_node(helper_node)
    executor.add_node(mirror)

    status_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    status_sub = helper_node.create_subscription(
        BEStatus,
        Topics._MIRROR_STATUS_TOPIC,
        observer.status_callback,
        status_qos,
    )

    stop_spin = threading.Event()
    profile_requested = threading.Event()
    profile_lock = threading.Lock()
    aggregate_profile = cProfile.Profile() if args.profile_output else None
    profile_active = False

    def _spin_executor():
        nonlocal profile_active
        while not stop_spin.is_set():
            try:
                if aggregate_profile is not None:
                    want_profile = profile_requested.is_set()
                    if want_profile and not profile_active:
                        with profile_lock:
                            aggregate_profile.enable()
                        profile_active = True
                    elif not want_profile and profile_active:
                        with profile_lock:
                            aggregate_profile.disable()
                        profile_active = False
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break
            except InvalidHandle:
                if stop_spin.is_set() or not rclpy.ok():
                    break
                time.sleep(0.01)
                continue
            except RuntimeError:
                if stop_spin.is_set() or not rclpy.ok():
                    break
                raise

    spin_thread = threading.Thread(target=_spin_executor, daemon=True)
    spin_thread.start()

    _write_tty_status('Mirror profiler starting up and waiting for onboard publishers...')
    try:
        if not _wait_for_match(lambda: mirror._status_sub.get_publisher_count() > 0, args.startup_timeout):
            raise RuntimeError('External onboard status publisher did not appear during startup')
        if not _wait_for_match(lambda: mirror._onboard_heartbeat_sub.get_publisher_count() > 0, args.startup_timeout):
            raise RuntimeError('External onboard heartbeat publisher did not appear during startup')
        if not _wait_for_match(lambda: mirror._mirror_status_pub.get_subscription_count() > 0, args.startup_timeout):
            raise RuntimeError('Mirror status subscriber did not match mirror status publisher during startup')
        if not observer.wait_for_ready(timeout_sec=args.startup_timeout):
            raise RuntimeError('Mirror did not publish READY during startup')
        if not _wait_for_match(lambda: _is_mirror_quiescent(mirror), args.startup_timeout):
            raise RuntimeError('Mirror was not idle at startup')

        _write_tty_status('Mirror profiler ready and waiting for onboard runs...')

        timings = []
        terminal_to_stopped_gaps = []
        stopped_to_ready_gaps = []
        status_lengths = []
        total_runs = args.warmup_runs + args.runs
        for run_index in range(total_runs):
            if run_index < args.warmup_runs:
                _write_tty_status(f'Warmup {run_index + 1} of {args.warmup_runs} ...')
            else:
                measured_index = run_index - args.warmup_runs + 1
                _write_tty_status(f'Running {measured_index} of {args.runs} ...')
                profile_requested.set()

            phase = 'warmup' if run_index < args.warmup_runs else 'measured'
            run_state = observer.collect_next_run(timeout_sec=args.timeout)
            if run_index >= args.warmup_runs:
                profile_requested.clear()

            if not run_state['started']:
                _emit_output_line(
                    f'run={run_index + 1:02d} phase={phase:8s} result=timeout '
                    f'note=timed_out_waiting_started status_count={run_state["status_count"]}'
                )
                break

            if run_state['terminal_time'] is None:
                _emit_output_line(
                    f'run={run_index + 1:02d} phase={phase:8s} result=timeout '
                    f'note=timed_out_waiting_terminal status_count={run_state["status_count"]}'
                )
                break

            if run_state['stopped_time'] is None:
                note = 'STARTED_before_STOPPED_after_terminal' if run_state['boundary'] == 'restarted' \
                    else 'missing_STOPPED_after_terminal'
                _emit_output_line(
                    f'run={run_index + 1:02d} phase={phase:8s} result=aborted '
                    f'note={note} status_count={run_state["status_count"]} last_status={run_state["last_code"]}'
                )
                break

            if run_state['ready_time'] is None:
                note = 'STARTED_before_READY_after_stopped' if run_state['boundary'] == 'restarted' \
                    else 'missing_READY_after_stopped'
                _emit_output_line(
                    f'run={run_index + 1:02d} phase={phase:8s} result=aborted '
                    f'note={note} status_count={run_state["status_count"]} last_status={run_state["last_code"]}'
                )
                break

            status_note = run_state['status_note']
            if status_note != 'ok':
                _emit_output_line(
                    f'run={run_index + 1:02d} phase={phase:8s} result=aborted '
                    f'note={status_note} status_count={run_state["status_count"]} last_status={run_state["last_code"]}'
                )
                continue

            duration = max(0.0, run_state['ready_time'] - run_state['started_at'])
            terminal_to_stopped_gap = max(0.0, run_state['stopped_time'] - run_state['terminal_time'])
            ready_gap = max(0.0, run_state['ready_time'] - run_state['stopped_time'])
            result_label = 'finished' if run_state['terminal_status'] == BEStatus.FINISHED else 'failed'

            _emit_output_line(
                f'run={run_index + 1:02d} phase={phase:8s} result={result_label:8s} '
                f'duration={duration:.6f}s '
                f'post_terminal_to_stopped={terminal_to_stopped_gap:.6f}s '
                f'post_stopped_to_ready={ready_gap:.6f}s '
                f'status_count={run_state["status_count"]}'
            )

            if run_index < args.warmup_runs:
                _write_tty_status(
                    f'Warmup {run_index + 1} of {args.warmup_runs} completed in {duration:.1f} seconds'
                )
            else:
                measured_index = run_index - args.warmup_runs + 1
                _write_tty_status(f'Run {measured_index} of {args.runs} completed in {duration:.1f} seconds')

            if run_index >= args.warmup_runs:
                timings.append(duration)
                terminal_to_stopped_gaps.append(terminal_to_stopped_gap)
                stopped_to_ready_gaps.append(ready_gap)
                status_lengths.append(run_state['status_count'])

        _write_tty_status('Mirror runs completed; processing timing summaries...')

        if aggregate_profile is not None:
            with profile_lock:
                if profile_active:
                    aggregate_profile.disable()
                    profile_active = False
            _emit_profile_summary(aggregate_profile, args.profile_output,
                                  args.profile_sort, args.profile_top)

        if timings:
            timing_min, timing_max, timing_average, timing_stddev = _timing_summary(timings)
            _emit_output_line(
                f'measured_runs={len(timings)} min={timing_min:.6f}s '
                f'max={timing_max:.6f}s avg={timing_average:.6f}s std={timing_stddev:.6f}s'
            )
            stop_min, stop_max, stop_average, stop_stddev = _timing_summary(terminal_to_stopped_gaps)
            _emit_output_line(
                f'phase_post_terminal_to_stopped: min={stop_min:.6f}s max={stop_max:.6f}s '
                f'avg={stop_average:.6f}s std={stop_stddev:.6f}s'
            )
            ready_min, ready_max, ready_average, ready_stddev = _timing_summary(stopped_to_ready_gaps)
            _emit_output_line(
                f'phase_post_stopped_to_ready: min={ready_min:.6f}s max={ready_max:.6f}s '
                f'avg={ready_average:.6f}s std={ready_stddev:.6f}s'
            )
            _emit_output_line(
                f'status_count: min={min(status_lengths)} max={max(status_lengths)} '
                f'avg={sum(status_lengths) / len(status_lengths):.6f}'
            )

        _emit_output_line(f'recorded_runs={args.warmup_runs + len(timings)}')
        _write_tty_status('Mirror profiler processing complete.')

    finally:
        stop_spin.set()
        try:
            mirror.shutdown_mirror()
        except (AttributeError, InvalidHandle, RuntimeError) as exc:
            _write_tty_status(f'Mirror shutdown cleanup error: {type(exc).__name__}: {exc}')
        helper_node.destroy_subscription(status_sub)
        mirror.destroy_node()
        helper_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
