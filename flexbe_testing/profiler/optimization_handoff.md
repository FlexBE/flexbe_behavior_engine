# Profiler Optimization Handoff

## Update: March 14, 2026 12:10 EDT

Latest `codex-mirror-circling` split-process reruns indicate that the current mirror profiler setup is finally usable and much less intrusive than the old phase-based harness.

Current reference files:

- `/tmp/jazzy_codex-mirror-circling_mirror.out`
- `/tmp/jazzy_codex-mirror-circling_mirror.err`
- `/tmp/jazzy_codex-mirror-circling_onboard.out`
- `/tmp/jazzy_codex-mirror-circling_onboard.err`
- `/tmp/jazzy_codex-mirror-circling_mirror2.out`
- `/tmp/jazzy_codex-mirror-circling_mirror2.err`
- `/tmp/jazzy_codex-mirror-circling_onboard2.out`
- `/tmp/jazzy_codex-mirror-circling_onboard2.err`

Key conclusions:

- The current mirror profiler, driven only by `flexbe/mirror/status` plus one coarse aggregate `cProfile`, completes cleanly on both the original and larger synthetic behaviors.
- The old phase-based mirror profiler was clearly far more intrusive. Comparing the current coarse profile against the legacy phase artifacts in `/tmp/mirror_split.prof.build.prof`, `/tmp/mirror_split.prof.spin.prof`, and `/tmp/mirror_split.prof.stop.prof` shows a large inflation in call volume and callback churn under the old `spin` profiler path.
- Both the current small and large runs still point to the same high-level cost structure: most time is in `rclpy` executor / waitable / callback machinery, not in mirror-specific Python code.
- Mirror-specific functions such as `_mirror_structure_callback()`, `_activate_mirror()`, `_mirror_state_machine()`, and `_drain_buffered_outcomes()` are visible in the current profile, but none of them stand out as dominant optimization targets.

Observed behavior in the latest runs:

- Small run: mirror status stream stayed clean with `status_count=63` and tightly clustered durations.
- Large run: mirror status stream stayed clean with `status_count=243` and similarly tight timing spread.
- Minor anomalies remain, but they did not break completion or accounting:
  - occasional `READY ... graceful stop already in progress`
  - one missed idle barrier warning on a small run
  - transient heartbeat divergence warnings on the large run that recovered cleanly
  - one onboard accounting oddity where the final zero outcome barrier was not counted on the last large run

Recommendation going forward:

- Treat the current mirror profiler as the baseline tool.
- Do not resurrect the old phase-based mirror profiling architecture.
- If deeper mirror optimization is needed later, focus first on reducing callback/executor churn or message volume rather than micro-optimizing mirror-specific Python code.

## Update: March 11, 2026 18:51 EDT

Question raised: should mirror publish profiler flags on existing topics, or add a dedicated mirror lifecycle topic?

Recommendation:

- Prefer a dedicated `MirrorStatus` topic over piggyback flags on `_BEHAVIOR_UPDATE_TOPIC` / `_STATE_MAP_OCS_TOPIC`.
- Keep mirror runtime behavior unchanged; this is observability only.
- Existing topics should remain semantically clean for normal consumers.

Why:

- current profiler run accounting fails when one external run produces multiple internal execute cycles due to normal mirror preempt/resync behavior
- explicit lifecycle messages provide robust run boundaries without heuristic matching
- this helps profiling and debugging, even if mirror internally restarts several times in one run

Suggested `MirrorStatus` fields (minimal):

- `run_token` (increment when STARTED is accepted)
- `lifecycle_state` (`READY`, `STARTING`, `RUNNING`, `STOPPING`, `IDLE`)
- `execute_cycle` (increment each `_execute_mirror` entry)
- `stop_reason` (`finished`, `preempt`, `error`, `timeout`, `unknown`)
- `behavior_id` / checksum
- timestamp

Expected profiler behavior with this telemetry:

- aggregate all internal execute cycles sharing one `run_token` into a single run record
- include preempt/resync cost in timings rather than filtering it out
- avoid false short-run completions and subsequent timeout drift

## Update: March 11, 2026 18:44 EDT

Context from latest `rolling_codex-refactor` reruns: mirror profiler still degrades under normal mirror resync/preempt behavior, and the current harness logic can drift run accounting when mirror executes multiple short internal start/stop cycles for one external run.

Key principle for next pass:

- do not alter mirror behavior for profiling
- profile must be passive instrumentation only
- if mirror preempts/resyncs in normal operation, timing output must include that cost rather than trying to "clean it up"

Current diagnosis:

- Instrumenting `_execute_mirror()` alone is not enough because there are often multiple execute completions per external run during forced stop/start recovery.
- The previous "next execute finished == next run" assumption causes short false run completions, then later timeout/hang.
- Additional external observer gating reduced some drift but added complexity and still failed in churn-heavy runs.

Proposed simplification for resume:

1. Keep all instrumentation inside `InstrumentedFlexbeMirror` (passive wrappers only on `_mirror_state_machine`, `_execute_mirror`, `_stop_mirror`, and status callback path).
2. Build a minimal internal run FSM keyed by observed status sequence:
   `READY -> STARTED -> (RUNNING)* -> FINISHED -> READY`
3. Aggregate all internal mirror cycles that occur between a given `STARTED` and terminal `READY` into one run record.
4. Mark runs invalid (aborted/timeout) when unexpected status ordering appears, but do not crash the profiler loop.
5. Remove ad-hoc cross-observer matching from the main loop once the internal FSM record stream is trustworthy.

Files/logs to resume from:

- `flexbe_testing/profiler/profile_mirror_process.py` (currently modified, uncommitted)
- `/tmp/rolling_codex-refactor_mirror.out`
- `/tmp/rolling_codex-refactor_mirror.err`

## Update: March 10, 2026 11:57

The startup stale-outcome problem is fixed, the mirror profiler harness race is fixed, and transient heartbeat mismatch instrumentation is now available. The remaining mirror issue is no longer hidden: in some runs the mirror falls onto the wrong branch and stays there while onboard continues through later branches.

Current reference files:

- `/tmp/jazzy_codex-refactor_onboard.out`
- `/tmp/jazzy_codex-refactor_onboard.err`
- `/tmp/jazzy_codex-refactor_mirror.out`
- `/tmp/jazzy_codex-refactor_mirror.err`

Recent relevant commits before the current uncommitted changes:

- `689a68d` Add outcome barrier handling for mirror profiler runs
- `be2d3d2` Reduce mirror sync false positives and fix profiler run races

### What Is Fixed

- raw `0` outcome barrier is now observed by mirror
- `no state handled outcome`: `0`
- `deferred premature outcome`: `0`
- confirmed `OCS is possibly out of sync`: `0`
- mirror profiler no longer times out waiting for a build due to per-run recorder races

### Current Mirror Failure Pattern

The transient mismatch instrumentation shows two different classes of remaining problems:

1. Same-branch one-step lag

Examples from `/tmp/jazzy_codex-refactor_mirror.err`:

- onboard `branch_1/.../level_1_step_0`, mirror still `branch_1/level_0_step_4`
- onboard `level_2_step_4`, mirror `level_2_step_3`
- onboard `level_3_step_1`, mirror `level_3_step_0`

Representative log locations:

- `/tmp/jazzy_codex-refactor_mirror.err` lines 18-28
- `/tmp/jazzy_codex-refactor_mirror.err` lines 73-85
- `/tmp/jazzy_codex-refactor_mirror.err` lines 100-113

2. Wrong-branch persistence

This is the more serious remaining bug.

In one measured run, onboard advances into branch 1 and later branches, but mirror stays on branch 0:

- onboard `branch_1/level_0_step_0`, mirror `branch_0/level_0_step_0`
- later onboard `branch_1/.../level_1_step_5`, mirror still `branch_0/level_0_step_0`
- later onboard `branch_2/...`, `branch_3/...`, and `branch_4/...`, mirror still reports branch 0 state

Representative log locations:

- `/tmp/jazzy_codex-refactor_mirror.err` lines 373-382
- `/tmp/jazzy_codex-refactor_mirror.err` lines 384-408
- `/tmp/jazzy_codex-refactor_mirror.err` lines 409-446
- `/tmp/jazzy_codex-refactor_mirror.err` lines 447-483

This means the remaining problem is not just harmless one-heartbeat jitter. The mirror can fail to advance into the correct branch after a run/lifecycle boundary or after a top-level/container transition.

### Additional Symptoms

- `Mirror finished spin with result 'None'` still happens every run.
- Mirror status capture is smeared across lifecycle boundaries:
  - onboard runs consistently report `statuses=[0, 1, 20]`
  - mirror runs can report `statuses=[0, 1, 20, 0]` or even `[1, 20]`
- mirror performance variance is now large, which is likely a symptom of lifecycle/sync issues rather than a real steady-state cost shift

From `/tmp/jazzy_codex-refactor_mirror.out`:

- measured duration min/max/avg/std = `8.888064 / 12.986501 / 11.242885 / 1.365475` seconds
- stop phase max = `0.723824` seconds

Onboard remains stable in `/tmp/jazzy_codex-refactor_onboard.out`.

### Most Likely Next Target

The next investigation target should be mirror lifecycle/start-stop interaction, not stale outcome buffering.

Primary files:

- [flexbe_mirror.py](/home/david/synth-test/src/flexbe_behavior_engine/flexbe_mirror/flexbe_mirror/flexbe_mirror.py)
- [mirror_state_machine.py](/home/david/synth-test/src/flexbe_behavior_engine/flexbe_mirror/flexbe_mirror/mirror_state_machine.py)

Focus areas:

- `_status_callback`
- `_start_mirror`
- `_stop_mirror`
- `_execute_mirror`
- top-level `spin()` exit behavior when root returns `None`
- interaction between `FINISHED`, `READY`, next `STARTED`, and structure arrival

Working hypothesis:

- mirror sometimes exits or tears down late enough that the next run's branch/state progression starts while the previous mirror lifecycle is still unwinding
- once that happens, the new run may start from the wrong top-level active branch and remain transiently or persistently wrong for the rest of the run

### Verification Commands

Useful checks for the current branch state:

```bash
pytest flexbe_mirror/tests/test_mirror_error_paths.py -q
python -m flake8 --jobs=1 --config flexbe_mirror/.flake8 flexbe_mirror
pytest flexbe_onboard/tests/test_onboard.py -q -k "publish_outcome_barrier or publish_ready_status_reuses_cached_message or behavior_execution_aborts_when_rclpy_is_not_ok"
python -B -m py_compile \
  flexbe_testing/profiler/profile_mirror_process.py \
  flexbe_onboard/flexbe_onboard/flexbe_onboard.py \
  flexbe_onboard/tests/test_onboard.py
```

## Current State

The recent mirror and profiler changes improved startup/build cost and removed the previous fast-exit regressions, but the current large-behavior profiler run shows that the mirror is still not tracking onboard execution correctly.

Latest reference files:

- `/tmp/jazzy_refactor_onboard4.out`
- `/tmp/jazzy_refactor_onboard4.err`
- `/tmp/jazzy_refactor_mirror4.out`
- `/tmp/jazzy_refactor_mirror4.err`

Recent relevant commits on this branch:

- `28e38cf` Ignore stale startup outcomes in mirror state machine
- `bf1bb5d` Index mirror structure before recursive build
- `1af1d45` Cache mirror structure path metadata

## Current Problem

The mirror is still semantically wrong on the large `*4` behavior.

It runs to completion and no longer crashes, but it spends most of each run out of sync with onboard.

Evidence from `/tmp/jazzy_refactor_mirror4.err`:

- `OCS is possibly out of sync`: `113`
- `no state handled outcome`: `1418`
- `ignored premature outcome`: `76`
- `Transient OCS heartbeat divergence detected`: `12`
- `Mirror finished spin with result 'None'`: `7`

The mirror is alive and finishes every run, but it is not mirroring the correct active path for much of the run.

## Observed Failure Pattern

The mismatch pattern is consistent:

1. The mirror starts correctly and enters the expected initial branch.
2. Onboard advances into later branches and deeper children.
3. Mirror logs ignored premature container outcomes.
4. After that, many real outcome messages become `no state handled outcome`.
5. Heartbeat mismatch reports continue for the rest of the branch progression.

Representative examples from `/tmp/jazzy_refactor_mirror4.err`:

- mirror stays on the initial branch subtree while onboard moves to `/branch_1`, `/branch_2`, and `/branch_3`
- mirror repeatedly logs:
  - `ignored premature outcome index=0 while current state=... is active`
  - `MirrorStateMachine 'root' (0) spin() - no state handled outcome from ... outcome index=0`

This strongly suggests the current hardening is preventing bogus early exits, but it is also dropping or failing to retain the container/root outcome needed to advance the parent container once the local child becomes ready to exit.

## Likely Root Cause

The next real target is mirror container/top-level outcome handling in:

- [mirror_state_machine.py](/home/david/synth-test/src/flexbe_behavior_engine/flexbe_mirror/flexbe_mirror/mirror_state_machine.py)

The current behavior appears to be:

- if a parent/container outcome arrives while a child is still active, log and ignore it
- later, when the child path actually completes, there is no retained parent outcome to consume
- the mirror remains on the old subtree and falls behind heartbeat state

The likely fix direction is:

- do not drop premature container/root outcomes
- hold them until the local container is actually ready to consume them
- then apply them once `_current_state` is cleared or the parent is otherwise in its terminal waiting state

This is different from the earlier deferred stale-outcome replay attempt:

- the old replay logic tried to resurrect arbitrary old outcomes from an untagged stream and caused cross-run problems
- the current issue is narrower: a same-run parent/container outcome is arriving slightly before the local child unwinds

## Recommended Next Steps

### 1. Fix mirror parent/container outcome handling

Primary target:

- [mirror_state_machine.py](/home/david/synth-test/src/flexbe_behavior_engine/flexbe_mirror/flexbe_mirror/mirror_state_machine.py)

Specifically inspect:

- top-level outcome handling in `spin()`
- internal container outcome handling in `_execute_current_state_mirror()`
- where `_current_state` becomes `None`
- whether a parent/container outcome can be retained briefly instead of dropped

Desired behavior:

- stale cross-run outcomes must still be ignored
- same-run premature parent/container outcomes should be held until locally consumable

### 2. Add a regression test for this exact large-behavior failure mode

Current tests are good for startup regressions and stop/recovery behavior, but they do not cover:

- a deep child finishing
- parent/container outcome arriving slightly early
- later branch transitions on the same run

Add a focused mirror test that simulates:

1. active deep child
2. early parent/container outcome
3. child completion
4. parent advance succeeds

### 3. Only after mirror correctness is fixed, revisit performance

When the mirror is semantically correct again, the next onboard optimization target is:

- `UserData` churn in `/tmp/jazzy_refactor_onboard4.out`

Main execute hotspots there are now:

- `user_data.py:41(__init__)`
- `user_data.py:100(__setattr__)`
- `state_machine.py:161(_execute_current_state)`
- `proxy_publisher.py:174(publish)`

## Useful Profiling Notes

### Onboard execute profile

Main steady-state costs in `/tmp/jazzy_refactor_onboard4.out`:

- `event_state.py:66(_event_execute)`
- `user_data.py:41(__init__)`
- `user_data.py:100(__setattr__)`
- `state_machine.py:161(_execute_current_state)`
- `waitable.py:37(__add__)`
- `proxy_publisher.py:174(publish)`

### Mirror spin profile

Main steady-state costs in `/tmp/jazzy_refactor_mirror4.out`:

- `waitable.py:37(__add__)`
- `list.extend`
- `contextlib.py:543(_push_cm_exit)`
- `executors.py:614(can_execute)`
- `callback_groups.py:101(can_execute)`

These numbers are currently polluted by repeated mismatch handling, so do not optimize them until the mirror stays in sync.

## Verification Commands Used

Useful local checks for the current branch:

```bash
pytest flexbe_mirror/tests/test_mirror_error_paths.py -q
pytest flexbe_core/test/test_logger_lifecycle.py -q
python -m flake8 --jobs=1 --config flexbe_mirror/.flake8 flexbe_mirror
```

Note:

- `ament_flake8` / `pytest flexbe_mirror/tests -k flake8` may still fail in the sandbox because `flake8` tries to use multiprocessing semaphores.
- `python -m flake8 --jobs=1 --config flexbe_mirror/.flake8 flexbe_mirror` is the reliable local fallback here.
