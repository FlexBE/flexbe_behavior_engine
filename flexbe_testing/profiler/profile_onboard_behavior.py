#!/usr/bin/env python3

"""Run a synthetic behavior through FlexBE onboard for profiling."""

import argparse
import cProfile
import os
import sys
import threading
import time
import zlib
from io import StringIO
from pstats import Stats
from statistics import pstdev

from flexbe_core.core.topics import Topics
from flexbe_core.proxy import ProxyPublisher

from flexbe_msgs.msg import BEStatus, BehaviorSelection, BehaviorSync

from flexbe_onboard import FlexbeOnboard

import rclpy
from rclpy._rclpy_pybind11 import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile


PROFILE_BEHAVIOR_NAME = 'Onboard Profile Behavior'
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
PROFILE_DATA_DIR = os.path.join(TOOLS_DIR, 'onboard_profile_data')
PROFILE_PHASES = ('prepare', 'confirm', 'execute', 'cleanup')


def _timing_summary(values):
    """Return min/max/avg/std summary for a non-empty numeric sequence."""
    average = sum(values) / len(values)
    return min(values), max(values), average, pstdev(values)


def _wait_for_match(predicate, timeout_sec, interval_sec=0.05):
    """Wait until a pub/sub match predicate becomes true."""
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


class ProfileObserver:
    """Collect status and heartbeat information for one run."""

    def __init__(self):
        self.condition = threading.Condition()
        self.heartbeat_count = 0
        self.status_codes = []
        self.started = False
        self.finished = False
        self.failed = False
        self.stopped = False
        self.last_status = None
        self.terminal_time = None
        self.stopped_time = None
        self.ready_after_terminal_time = None

    def status_callback(self, msg):
        """Track onboard status messages."""
        with self.condition:
            self.last_status = msg.code
            self.status_codes.append(msg.code)
            if msg.code == BEStatus.STARTED:
                self.started = True
            elif msg.code == BEStatus.FINISHED:
                self.finished = True
                if self.terminal_time is None:
                    self.terminal_time = time.perf_counter()
            elif msg.code in (BEStatus.FAILED, BEStatus.ERROR):
                self.failed = True
                if self.terminal_time is None:
                    self.terminal_time = time.perf_counter()
            elif msg.code == BEStatus.WARNING:
                # WARNING is not terminal on onboard; keep waiting for FAILED/ERROR/FINISHED.
                pass
            elif msg.code == BEStatus.STOPPED:
                self.stopped = True
                if self.stopped_time is None:
                    self.stopped_time = time.perf_counter()
            elif msg.code == BEStatus.READY and self.terminal_time is not None and self.ready_after_terminal_time is None:
                self.ready_after_terminal_time = time.perf_counter()
            self.condition.notify_all()

    def heartbeat_callback(self, _msg):
        """Count heartbeat messages."""
        with self.condition:
            self.heartbeat_count += 1
            self.condition.notify_all()

    def reset(self):
        """Reset per-run counters."""
        with self.condition:
            self.heartbeat_count = 0
            self.status_codes = []
            self.started = False
            self.finished = False
            self.failed = False
            self.stopped = False
            self.last_status = None
            self.terminal_time = None
            self.stopped_time = None
            self.ready_after_terminal_time = None

    def wait_for_ready(self, timeout_sec):
        """Wait until READY is observed."""
        return self._wait_for(lambda: self.last_status == BEStatus.READY, timeout_sec)

    def wait_for_completion(self, timeout_sec):
        """Wait until a run has either finished or failed."""
        return self._wait_for(lambda: self.finished or self.failed, timeout_sec)

    def wait_for_stopped(self, timeout_sec):
        """Wait until STOPPED is observed after the terminal run status."""
        return self._wait_for(lambda: self.stopped, timeout_sec)

    def _wait_for(self, predicate, timeout_sec):
        end_time = time.monotonic() + timeout_sec
        with self.condition:
            while time.monotonic() < end_time:
                if predicate():
                    return True
                self.condition.wait(timeout=0.05)
        return predicate()


class PhaseRecorder:
    """Collect coarse-grained onboard execution phase timings."""

    def __init__(self):
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        """Reset all recorded phase timings."""
        with self._lock:
            self.prepare_duration = 0.0
            self.confirm_duration = 0.0
            self.execute_duration = 0.0
            self.cleanup_duration = 0.0
            self.thread_total_duration = 0.0
            self.post_execute_tail_duration = 0.0
            self.post_finish_to_ready_duration = 0.0
            self._execute_finished_at = None

    def record_prepare(self, duration):
        """Record behavior preparation duration."""
        with self._lock:
            self.prepare_duration += duration

    def record_confirm(self, duration):
        """Record behavior confirmation duration."""
        with self._lock:
            self.confirm_duration += duration

    def record_execute(self, duration):
        """Record behavior execution duration."""
        with self._lock:
            self.execute_duration += duration

    def record_execute_finished_at(self, timestamp):
        """Record the wall-clock boundary when behavior.execute() returned."""
        with self._lock:
            self._execute_finished_at = timestamp

    def record_cleanup(self, duration):
        """Record behavior cleanup duration."""
        with self._lock:
            self.cleanup_duration += duration

    def record_thread_total(self, duration):
        """Record full behavior-thread lifetime."""
        with self._lock:
            self.thread_total_duration += duration

    def record_post_execute_tail(self, duration):
        """Record time after behavior.execute() returns until thread exit."""
        with self._lock:
            self.post_execute_tail_duration += duration

    def record_post_finish_to_ready(self, duration):
        """Record time from terminal status until READY is observed."""
        with self._lock:
            self.post_finish_to_ready_duration += duration

    def snapshot(self):
        """Return the current phase totals."""
        with self._lock:
            return {
                'prepare': self.prepare_duration,
                'confirm': self.confirm_duration,
                'execute': self.execute_duration,
                'cleanup': self.cleanup_duration,
                'thread_total': self.thread_total_duration,
                'post_execute_tail': self.post_execute_tail_duration,
                'post_finish_to_ready': self.post_finish_to_ready_duration,
            }

    def get_execute_finished_at(self):
        """Return the recorded execute-finished timestamp for the current run."""
        with self._lock:
            return self._execute_finished_at


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=3, help='Number of profiling runs to execute.')
    parser.add_argument('--warmup-runs', type=int, default=1, help='Warmup runs before measurement.')
    parser.add_argument('--depth', type=int, default=6, help='Nested state machine depth.')
    parser.add_argument('--width', type=int, default=8, help='States per nested level.')
    parser.add_argument('--branches', type=int, default=4, help='Top-level branch count.')
    parser.add_argument('--payload', type=int, default=0, help='Tiny arithmetic loop inside each state.')
    parser.add_argument('--ticks-per-state', type=int, default=5,
                        help='How many execute() invocations each synthetic state requires before returning done.')
    parser.add_argument('--state-rate-hz', type=float, default=100.0,
                        help='Desired per-state execution rate for the synthetic profiling state.')
    parser.add_argument('--autonomy-level', type=int, default=255, help='BehaviorSelection autonomy level.')
    parser.add_argument('--timeout', type=float, default=30.0, help='Timeout per run in seconds.')
    parser.add_argument('--inter-run-delay', type=float, default=0.05,
                        help='Delay in seconds between completed runs before launching the next one.')
    parser.add_argument('--subscribe-heartbeat', action='store_true',
                        help='Attach a heartbeat subscriber so onboard publishes heartbeat traffic.')
    parser.add_argument('--publish-mirror-structure', action='store_true',
                        help='Publish the behavior structure just after confirm() for an external mirror profiler.')
    parser.add_argument('--enable-state-logging', action='store_true',
                        help='Enable state logger (`log_enabled:=True`) on onboard for overhead measurements.')
    parser.add_argument('--profile-output', default='',
                        help='Optional cProfile output file for onboard._behavior_execution().')
    parser.add_argument('--profile-sort', default='cumulative',
                        help='Sort key for printed cProfile stats (default: cumulative).')
    parser.add_argument('--profile-top', type=int, default=25,
                        help='How many cProfile rows to print.')
    return parser.parse_args()


def _build_request(onboard, args):
    behavior_key, _ = onboard._behavior_lib.find_behavior(PROFILE_BEHAVIOR_NAME)
    if behavior_key is None:
        raise RuntimeError(f"Unable to find '{PROFILE_BEHAVIOR_NAME}' after adding local manifests")

    source_path = onboard._behavior_lib.get_sourcecode_filepath(behavior_key)
    with open(source_path, encoding='utf-8') as handle:
        behavior_id = zlib.adler32(handle.read().encode()) & 0x7fffffff

    request = BehaviorSelection()
    request.behavior_key = behavior_key
    request.behavior_id = behavior_id
    request.autonomy_level = args.autonomy_level
    request.arg_keys = ['depth', 'width', 'branches', 'payload', 'ticks_per_state', 'state_rate_hz']
    request.arg_values = [
        str(args.depth),
        str(args.width),
        str(args.branches),
        str(args.payload),
        str(args.ticks_per_state),
        str(args.state_rate_hz)
    ]
    return request


def main():
    """Run the standalone onboard profiling harness."""
    args = _parse_args()
    sys.path.insert(0, TOOLS_DIR)

    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=max(2, os.cpu_count() or 2))
    helper_node = rclpy.create_node('onboard_profile_driver')
    onboard = FlexbeOnboard()
    if not onboard.has_parameter('log_enabled'):
        onboard.declare_parameter('log_enabled', args.enable_state_logging)
    onboard.set_parameters([Parameter('log_enabled', Parameter.Type.BOOL, args.enable_state_logging)])
    onboard.executor = executor
    observer = ProfileObserver()
    phase_recorder = PhaseRecorder()

    executor.add_node(helper_node)
    executor.add_node(onboard)

    status_qos = QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    status_sub = helper_node.create_subscription(BEStatus, Topics._ONBOARD_STATUS_TOPIC,
                                                 observer.status_callback, status_qos)
    heartbeat_sub = None
    if args.subscribe_heartbeat:
        heartbeat_sub = helper_node.create_subscription(
            BehaviorSync, Topics._ONBOARD_HEARTBEAT_TOPIC, observer.heartbeat_callback, QoSProfile(depth=50)
        )

    behavior_pub = helper_node.create_publisher(BehaviorSelection, Topics._START_BEHAVIOR_TOPIC, 10)

    phase_profiles = {phase_name: [] for phase_name in PROFILE_PHASES}
    profile_lock = threading.Lock()
    profile_session_lock = threading.Lock()
    original_prepare_behavior = onboard._prepare_behavior
    original_cleanup_behavior = onboard._cleanup_behavior
    original_behavior_execution = onboard._behavior_execution

    def _timed_prepare_behavior(request):
        start_time = time.perf_counter()
        local_profiler = cProfile.Profile() if args.profile_output else None
        if local_profiler is not None:
            local_profiler.enable()
        try:
            behavior = original_prepare_behavior(request)
        finally:
            if local_profiler is not None:
                local_profiler.disable()
                with profile_lock:
                    phase_profiles['prepare'].append(local_profiler)
        phase_recorder.record_prepare(time.perf_counter() - start_time)
        if behavior is None:
            return None

        original_confirm = behavior.confirm

        def _timed_confirm():
            confirm_start = time.perf_counter()
            local_profiler = cProfile.Profile() if args.profile_output else None
            if local_profiler is not None:
                local_profiler.enable()
            try:
                result = original_confirm()
            finally:
                if local_profiler is not None:
                    local_profiler.disable()
                    with profile_lock:
                        phase_profiles['confirm'].append(local_profiler)
                phase_recorder.record_confirm(time.perf_counter() - confirm_start)
            if args.publish_mirror_structure:
                structure_msg = getattr(getattr(behavior, '_state_machine', None), '_structure', None)
                if structure_msg is not None:
                    behavior._state_machine._pub.publish(Topics._MIRROR_STRUCTURE_TOPIC, structure_msg)
                    # Give an external mirror process a brief head start before STARTED is published.
                    time.sleep(0.05)
            return result

        original_execute = behavior.execute

        def _timed_execute():
            execute_start = time.perf_counter()
            local_profiler = cProfile.Profile() if args.profile_output else None
            if local_profiler is not None:
                local_profiler.enable()
            try:
                return original_execute()
            finally:
                execute_end = time.perf_counter()
                if local_profiler is not None:
                    local_profiler.disable()
                    with profile_lock:
                        phase_profiles['execute'].append(local_profiler)
                phase_recorder.record_execute(execute_end - execute_start)
                phase_recorder.record_execute_finished_at(execute_end)

        behavior.confirm = _timed_confirm
        behavior.execute = _timed_execute
        return behavior

    def _timed_cleanup_behavior(behavior_id):
        cleanup_start = time.perf_counter()
        local_profiler = cProfile.Profile() if args.profile_output else None
        if local_profiler is not None:
            local_profiler.enable()
        try:
            return original_cleanup_behavior(behavior_id)
        finally:
            if local_profiler is not None:
                local_profiler.disable()
                with profile_lock:
                    phase_profiles['cleanup'].append(local_profiler)
            phase_recorder.record_cleanup(time.perf_counter() - cleanup_start)

    onboard._prepare_behavior = _timed_prepare_behavior
    onboard._cleanup_behavior = _timed_cleanup_behavior

    def _profiled_behavior_execution(request):
        thread_start = time.perf_counter()
        with profile_session_lock:
            try:
                return original_behavior_execution(request)
            finally:
                thread_end = time.perf_counter()
                phase_recorder.record_thread_total(thread_end - thread_start)
                execute_finished_at = phase_recorder.get_execute_finished_at()
                if execute_finished_at is not None:
                    phase_recorder.record_post_execute_tail(max(0.0, thread_end - execute_finished_at))

    onboard._behavior_execution = _profiled_behavior_execution

    onboard._behavior_lib._add_behavior_manifests(PROFILE_DATA_DIR)
    request = _build_request(onboard, args)

    stop_spin = threading.Event()

    def _spin_executor():
        while not stop_spin.is_set():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break
            except InvalidHandle:
                if stop_spin.is_set() or not rclpy.ok():
                    break
                # Tear-down can invalidate waitables while the executor thread is unwinding.
                time.sleep(0.01)
                continue

    spin_thread = threading.Thread(target=_spin_executor, daemon=True)
    spin_thread.start()
    output_lines = []
    phase_stats_output = []

    try:
        _write_tty_status('Onboard profiler starting up and waiting for ROS topic matches...')

        if not _wait_for_match(lambda: onboard._status_pub.get_subscription_count() > 0, args.timeout):
            raise RuntimeError('Status subscriber did not match onboard status publisher during startup')

        if observer.last_status != BEStatus.READY:
            onboard._publish_ready_status()

        if not _wait_for_match(lambda: onboard._start_beh_sub.get_publisher_count() > 0, args.timeout):
            raise RuntimeError('Behavior request publisher did not match onboard start subscriber during startup')

        if heartbeat_sub is not None:
            if not _wait_for_match(lambda: onboard._heartbeat_pub.get_subscription_count() > 0, args.timeout):
                raise RuntimeError('Heartbeat subscriber did not match onboard heartbeat publisher during startup')

        if not observer.wait_for_ready(timeout_sec=args.timeout):
            raise RuntimeError('Onboard did not publish READY during startup')

        if args.publish_mirror_structure:
            _write_tty_status('Onboard profiler ready to start runs; external mirror may connect on structure/outcome topics.')
        else:
            _write_tty_status('Onboard profiler ready to start runs.')

        timings = []
        phase_history = {
            'prepare': [],
            'confirm': [],
            'execute': [],
            'cleanup': [],
            'thread_total': [],
            'post_execute_tail': [],
            'post_finish_to_ready': [],
        }
        total_runs = args.warmup_runs + args.runs
        for run_index in range(total_runs):
            if run_index < args.warmup_runs:
                _write_tty_status(f'Warmup {run_index + 1} of {args.warmup_runs} ...')
            else:
                measured_index = run_index - args.warmup_runs + 1
                _write_tty_status(f'Running {measured_index} of {args.runs} ...')

            observer.reset()
            with profile_session_lock:
                # Ensure no prior behavior-execution thread can write into a freshly reset recorder.
                phase_recorder.reset()
            ProxyPublisher.reset_outcome_publish_counters()
            start_time = time.perf_counter()
            behavior_pub.publish(request)

            if not observer.wait_for_completion(timeout_sec=args.timeout):
                raise RuntimeError(f'Run {run_index + 1} timed out waiting for completion')

            duration = time.perf_counter() - start_time
            result = 'finished' if observer.finished else 'failed'
            if observer.failed:
                raise RuntimeError(
                    f'Run {run_index + 1} failed with statuses={observer.status_codes}'
                )

            if not observer.wait_for_stopped(timeout_sec=args.timeout):
                raise RuntimeError(f'Run {run_index + 1} did not publish STOPPED')

            if not observer.wait_for_ready(timeout_sec=args.timeout):
                raise RuntimeError(f'Run {run_index + 1} did not return to READY')

            if observer.terminal_time is not None and observer.ready_after_terminal_time is not None:
                ready_gap = observer.ready_after_terminal_time - observer.terminal_time
                phase_recorder.record_post_finish_to_ready(ready_gap)

            heartbeat_count = observer.heartbeat_count
            status_codes = list(observer.status_codes)
            with profile_session_lock:
                # Wait for any in-flight behavior execution finalize path before sampling phase totals.
                phase_timings = phase_recorder.snapshot()
            publish_counters = ProxyPublisher.get_outcome_publish_counters()
            phase = 'warmup' if run_index < args.warmup_runs else 'measured'
            output_lines.append(
                f'run={run_index + 1:02d} phase={phase:8s} result={result:8s} '
                f'duration={duration:.6f}s '
                f'prepare={phase_timings["prepare"]:.6f}s '
                f'confirm={phase_timings["confirm"]:.6f}s '
                f'execute={phase_timings["execute"]:.6f}s '
                f'cleanup={phase_timings["cleanup"]:.6f}s '
                f'outcome_pub_total={publish_counters["outcome_pub_total"]} '
                f'outcome_pub_nonzero={publish_counters["outcome_pub_nonzero"]} '
                f'heartbeats={heartbeat_count} statuses={status_codes}'
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
                for phase_name, phase_value in phase_timings.items():
                    phase_history[phase_name].append(phase_value)

            if run_index + 1 < total_runs and args.inter_run_delay > 0.0:
                time.sleep(args.inter_run_delay)

        _write_tty_status('Onboard runs completed; processing timing summaries and profile data...')

        if timings:
            timing_min, timing_max, timing_average, timing_stddev = _timing_summary(timings)
            output_lines.append(
                f'measured_runs={len(timings)} min={timing_min:.6f}s '
                f'max={timing_max:.6f}s avg={timing_average:.6f}s std={timing_stddev:.6f}s'
            )
            for phase_name in ('prepare', 'confirm', 'execute', 'cleanup',
                               'thread_total', 'post_execute_tail', 'post_finish_to_ready'):
                phase_values = phase_history[phase_name]
                phase_min, phase_max, phase_average, phase_stddev = _timing_summary(phase_values)
                output_lines.append(
                    f'phase_{phase_name}: min={phase_min:.6f}s '
                    f'max={phase_max:.6f}s avg={phase_average:.6f}s std={phase_stddev:.6f}s'
                )

        if args.profile_output:
            for phase_name in PROFILE_PHASES:
                if not phase_profiles[phase_name]:
                    continue
                phase_stats = Stats(phase_profiles[phase_name][0])
                for profiler in phase_profiles[phase_name][1:]:
                    phase_stats.add(profiler)
                phase_output_path = f'{args.profile_output}.{phase_name}.prof'
                phase_stats.dump_stats(phase_output_path)
                phase_stream = StringIO()
                phase_stats.strip_dirs().sort_stats(args.profile_sort)
                phase_stats.stream = phase_stream
                phase_stats.print_stats(args.profile_top)
                phase_stats_output.append(
                    f'phase profile {phase_name} written to {phase_output_path}\n'
                    f'{phase_stream.getvalue().rstrip()}'
                )

        _write_tty_status('Onboard profiler processing complete.')

    finally:
        stop_spin.set()
        if heartbeat_sub is not None:
            helper_node.destroy_subscription(heartbeat_sub)
        helper_node.destroy_subscription(status_sub)
        helper_node.destroy_publisher(behavior_pub)
        onboard.destroy_node()
        helper_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)

    for line in output_lines:
        print(line, flush=True)
    for phase_output in phase_stats_output:
        print(phase_output, flush=True)


if __name__ == '__main__':
    main()
