# Profiler Comparison: `rolling_codex-refactor` vs `rolling_ros2-devel-profiler`

## Inputs

Compared these output artifacts:

- `/tmp/rolling_codex-refactor_onboard.out`
- `/tmp/rolling_ros2-devel-profiler_onboard.out`
- `/tmp/rolling_codex-refactor_mirror.out`
- `/tmp/rolling_ros2-devel-profiler_mirror.out`
- `/tmp/rolling_codex-refactor_mirror.err`
- `/tmp/rolling_ros2-devel-profiler_mirror.err`

## Timing Summary (Measured Runs)

### Onboard

| Metric | codex-refactor | ros2-devel-profiler | Delta |
|---|---:|---:|---:|
| measured avg | 11.573845s | 12.328658s | +0.754813s (+6.52%) |
| phase_execute avg | 11.465901s | 11.771574s | +0.305673s (+2.67%) |
| phase_prepare avg | 0.020504s | 0.179052s | +0.158548s (+773.25%) |
| phase_confirm avg | 0.018411s | 0.289768s | +0.271357s (+1473.89%) |
| phase_cleanup avg | 0.005742s | 0.002562s | -0.003180s (-55.38%) |
| phase_thread_total avg | 11.564958s | 12.332613s | +0.767655s (+6.64%) |

### Mirror

| Metric | codex-refactor | ros2-devel-profiler | Delta |
|---|---:|---:|---:|
| measured avg | 11.541163s | 11.840657s | +0.299494s (+2.60%) |
| phase_spin avg | 11.486513s | 11.676075s | +0.189562s (+1.65%) |
| phase_build avg | 0.016621s | 0.047952s | +0.031331s (+188.50%) |
| phase_stop avg | 0.000701s | 0.011687s | +0.010986s (+1567.19%) |
| phase_thread_total avg | 11.487166s | 11.682647s | +0.195481s (+1.70%) |

## Observed Behavioral Differences

### Onboard

- `ros2-devel-profiler` shows much higher `prepare` and `confirm` cost.
- Its cProfile output includes heavy `state_logger` and logger caller-inspection overhead (`rcutils_logger`, `inspect`) in setup paths.
- Execute is also slower (+~0.31s average), with `state_logger` wrappers appearing in execute top functions.

### Mirror

- `ros2-devel-profiler` has more sync churn and warning traffic.
- In mirror stderr:
  - `OCS is possibly out of sync` appears 5 times (0 in codex-refactor).
  - `no state handled outcome ... outcome index=0` appears 27 times (0 in codex-refactor).
- Run summaries show `stop_count` often equals `2` on `ros2-devel-profiler`, while codex-refactor is consistently `1`.
- These correlate with increased `spin`, `build`, and especially `stop` timing.

## Takeaway

`rolling_codex-refactor` is faster and cleaner in both onboard and mirror runs.

Primary contributors to the slower `rolling_ros2-devel-profiler` runs are:

1. Extra setup/logging overhead (state logger + rcutils caller inspection) in onboard `prepare/confirm`.
2. Additional mirror sync recovery churn (out-of-sync checks and unhandled-outcome warnings), which increases `spin/stop` overhead.

## Deep Dive: Top Offenders by Version

`cProfile` cumulative time is used for ranking hotspots. Values below are the top per-phase offenders from each run's printed profile tables.

### `rolling_codex-refactor`

#### Onboard (phase split)

- `phase_execute` avg = `11.465901s` (`99.07%` of measured run time)
- `phase_prepare` avg = `0.020504s`
- `phase_confirm` avg = `0.018411s`
- `phase_cleanup` avg = `0.005742s`

#### Onboard execute top offenders

| Cum Time | Function |
|---:|---|
| 1.152s | `event_state.py:66(_event_execute)` |
| 0.934s | `context.py:179(on_shutdown)` |
| 0.591s | `user_data.py:41(__init__)` |
| 0.562s | `user_data.py:100(__setattr__)` |
| 0.444s | `state_machine.py:161(_execute_current_state)` |
| 0.397s | `time.py:37(__init__)` |
| 0.380s | `operatable_state.py:65(_operatable_execute)` |
| 0.360s | `preemptable_state.py:71(_preemptable_execute)` |

#### Mirror (phase split)

- `phase_spin` avg = `11.486513s` (`99.53%` of measured run time)
- `phase_build` avg = `0.016621s`
- `phase_stop` avg = `0.000701s`

#### Mirror spin top offenders

| Cum Time | Function |
|---:|---|
| 4.356s | `{method 'extend' of 'list' objects}` |
| 3.264s | `contextlib.py:543(_push_cm_exit)` |
| 2.819s | `executors.py:694(can_execute)` |
| 1.918s | `callback_groups.py:113(can_execute)` |
| 1.448s | `event_handler.py:168(__enter__)` |
| 1.169s | `contextlib.py:548(_push_exit_callback)` |
| 1.051s | `event_handler.py:159(get_num_entities)` |
| 0.685s | `contextlib.py:471(_create_exit_wrapper)` |

### `rolling_ros2-devel-profiler`

#### Onboard (phase split)

- `phase_execute` avg = `11.771574s` (`95.48%` of measured run time)
- `phase_prepare` avg = `0.179052s` (`1.45%`)
- `phase_confirm` avg = `0.289768s` (`2.35%`)
- `phase_cleanup` avg = `0.002562s`

#### Onboard prepare/confirm top offenders

| Cum Time | Function |
|---:|---|
| 0.445s | `state_logger.py:235(log_userdata_init)` |
| 0.436s | `state_logger.py:208(log_outcomes_init)` |
| 0.379s | `state_logger.py:176(log_events_init)` |
| 0.351s | `{method 'acquire' of '_thread.lock' objects}` |
| 0.201s | `threading.py:323(wait)` |
| 0.133s | `clock.py:134(__exit__)` |
| 0.117s | `node.py:1516(add_waitable)` |

#### Onboard execute top offenders

| Cum Time | Function |
|---:|---|
| 1.256s | `event_state.py:66(_event_execute)` |
| 0.885s | `context.py:179(on_shutdown)` |
| 0.714s | `{method 'extend' of 'list' objects}` |
| 0.593s | `user_data.py:41(__init__)` |
| 0.562s | `user_data.py:103(__setattr__)` |
| 0.522s | `ros_state.py:83(sleep_duration)` |
| 0.493s | `executors.py:694(can_execute)` |
| 0.459s | `contextlib.py:543(_push_cm_exit)` |

#### Mirror (phase split)

- `phase_spin` avg = `11.676075s` (`98.61%` of measured run time)
- `phase_build` avg = `0.047952s`
- `phase_stop` avg = `0.011687s`

#### Mirror build/spin/stop top offenders

| Cum Time | Function |
|---:|---|
| 0.061s | `state_logger.py:235(log_userdata_init)` (build) |
| 0.051s | `state_logger.py:176(log_events_init)` (build) |
| 0.048s | `state_logger.py:180(wrap_event_method)` (build) |
| 2.882s | `{method 'extend' of 'list' objects}` (spin) |
| 1.919s | `executors.py:694(can_execute)` (spin) |
| 1.580s | `callback_groups.py:113(can_execute)` (spin) |
| 1.389s | `contextlib.py:543(_push_cm_exit)` (spin) |
| 1.372s | `event_handler.py:168(__enter__)` (spin) |
| 0.591s | `proxy_subscriber_cached.py:334(has_buffered)` (spin) |
| 0.006s | `logger.py:85(log)` (stop) |
| 0.005s | `inspect.py:1677(getframeinfo)` (stop) |
