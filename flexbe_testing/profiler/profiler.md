# Profiling

This folder contains standalone profiling harnesses for `flexbe_onboard` and `flexbe_mirror`.
The default and recommended way to profile mirror is the split-process workflow below.

## Onboard Profiling

`profile_onboard_behavior.py` starts a real `FlexbeOnboard` node, injects a synthetic local behavior, runs that behavior multiple times, and reports:

- per-run total duration
- per-run phase timings:
  - `prepare`
  - `confirm`
  - `execute`
  - `cleanup`
  - `thread_total`
  - `post_execute_tail`
  - `post_finish_to_ready`
- measured-run min/max/average summaries
- measured-run standard deviation summaries
- optional phase-specific `cProfile` output

The synthetic behavior is stored in `onboard_profile_data/` and is designed to stress:

- nested state-machine construction
- behavior confirmation/setup
- execution through many small states over multiple `execute()` ticks per state
- optional heartbeat traffic when `--subscribe-heartbeat` is used
- optional state logger overhead when `--enable-state-logging` is used
- optional pause between runs via `--inter-run-delay`

Shape parameters:

- `--depth` controls how many nested state-machine levels each branch contains.
- `--width` controls how many states are placed at each nested level before descending to the next level.
- `--branches` controls how many top-level branches exist under the root state machine.

Example for `--branches 2 --depth 4 --width 3`:

```text
root
|-- branch_0
|   |-- level_0: [s0] -> [s1] -> [s2] -> level_0_sub_sm
|   |   |-- level_1: [s0] -> [s1] -> [s2] -> level_1_sub_sm
|   |   |   |-- level_2: [s0] -> [s1] -> [s2] -> level_2_sub_sm
|   |   |   |   `-- level_3: [s0] -> [s1] -> [s2]
|
`-- branch_1
    |-- level_0: [s0] -> [s1] -> [s2] -> level_0_sub_sm
    |   |-- level_1: [s0] -> [s1] -> [s2] -> level_1_sub_sm
    |   |   |-- level_2: [s0] -> [s1] -> [s2] -> level_2_sub_sm
    |   |   |   `-- level_3: [s0] -> [s1] -> [s2]
```

Read it as:

- `branches=2`: `root` has `branch_0` and `branch_1`
- `depth=4`: each branch contains `level_0`, `level_1`, `level_2`, and `level_3`
- `width=3`: each level runs three states before entering the next nested level
- `level_N_sub_sm`: the nested child state machine entered after that level's width states complete

Each synthetic state:

- runs at a configurable state tick rate via `--state-rate-hz`
- does a small amount of work per tick via `--payload`
- returns `done` only after `--ticks-per-state` `execute()` calls

That makes the profile more representative than a single blocking `execute()` call.

## Example

Copy and run:

```bash
python3 flexbe_testing/profiler/profile_onboard_behavior.py \
  --warmup-runs 2 \
  --runs 5 \
  --depth 6 \
  --width 8 \
  --branches 4 \
  --ticks-per-state 10 \
  --state-rate-hz 100.0 \
  --subscribe-heartbeat \
  --profile-output /tmp/onboard.prof
```

This will:

- perform 2 warmup runs
- measure 5 runs
- execute each synthetic state for 10 ticks before completion
- run states at 100 Hz
- pause 0.05 seconds between runs by default
- enable heartbeat traffic
- keep onboard state logging disabled by default (`log_enabled:=False`)
- write phase-specific `cProfile` stats rooted at `/tmp/onboard.prof`

### State Logger Overhead

`profile_onboard_behavior.py` defaults to state logging disabled to measure core execution cost.
To include state logger runtime overhead in a test run, add:

```bash
--enable-state-logging
```

When enabled, expect higher execute-phase CPU and additional logging-related stack activity (`rcutils_logger`, `inspect`).

### Inter-Run Delay

`profile_onboard_behavior.py` now waits `0.05` seconds between runs by default. This gives external consumers such as the mirror profiler a small quiet period between behavior executions.

To override it, add for example:

```bash
--inter-run-delay 0.2
```

## Useful Variants

Smaller quick check:

```bash
python3 flexbe_testing/profiler/profile_onboard_behavior.py \
  --warmup-runs 1 \
  --runs 3 \
  --depth 3 \
  --width 3 \
  --ticks-per-state 5 \
  --state-rate-hz 100.0
```

Print more profile rows:

```bash
python3 flexbe_testing/profiler/profile_onboard_behavior.py \
  --profile-output /tmp/onboard.prof \
  --profile-top 100
```

Sort profile output by self time:

```bash
python3 flexbe_testing/profiler/profile_onboard_behavior.py \
  --profile-output /tmp/onboard.prof \
  --profile-sort tottime
```

Suggested copy-paste command:

```bash
python3 flexbe_testing/profiler/profile_onboard_behavior.py \
  --warmup-runs 2 \
  --runs 5 \
  --depth 3 \
  --width 3 \
  --ticks-per-state 10 \
  --state-rate-hz 100.0 \
  --subscribe-heartbeat \
  --profile-output /tmp/onboard.prof
```

## Mirror Profiling

`profile_mirror_process.py` profiles only `FlexbeMirror` and waits for an external onboard run to drive it through the normal FlexBE topics. This is the default mirror profiling setup because it avoids same-process onboard executor noise.

Use two terminals.

Terminal 1: start the mirror-only profiler first.

```bash
clear; python3 flexbe_testing/profiler/profile_mirror_process.py \
  --warmup-runs 1 \
  --runs 4 \
  --startup-timeout 40 \
  --timeout 30 \
  --profile-output /tmp/mirror_split.prof \
  > "/tmp/${ROS_DISTRO}_$(git rev-parse --abbrev-ref HEAD | tr '/' '_')_mirror.out" \
  2> "/tmp/${ROS_DISTRO}_$(git rev-parse --abbrev-ref HEAD | tr '/' '_')_mirror.err"
```

Terminal 2: start the onboard profiler with early structure publication enabled.

```bash
clear; python3 flexbe_testing/profiler/profile_onboard_behavior.py \
  --warmup-runs 1 \
  --runs 4 \
  --branches 4 \
  --depth 3 \
  --width 4 \
  --ticks-per-state 10 \
  --state-rate-hz 100.0 \
  --publish-mirror-structure \
  --profile-output /tmp/onboard.prof \
  > "/tmp/${ROS_DISTRO}_$(git rev-parse --abbrev-ref HEAD | tr '/' '_')_onboard.out" \
  2> "/tmp/${ROS_DISTRO}_$(git rev-parse --abbrev-ref HEAD | tr '/' '_')_onboard.err"
```

Notes:

- Start the mirror profiler before the onboard profiler.
- Keep `--warmup-runs` and `--runs` aligned between the two terminals.
- `--publish-mirror-structure` makes the onboard profiler publish the behavior structure just after `confirm()` so the external mirror process can build before `BEStatus.STARTED`.
- With the commands above, one run produces mirror status-based timing output in `mirror.out`, a single coarse aggregate mirror `cProfile` file at `/tmp/mirror_split.prof`, and onboard phase profiles under `/tmp/onboard.prof.*.prof`.
- In this split-process mode, `profile_mirror_process.py` uses `flexbe/mirror/status` as the run boundary source.
- Expected mirror status flow per run is `STARTED -> (RUNNING|WARNING)* -> FINISHED -> READY`.
- The harness reports externally observed run timing from `STARTED` to `READY`, the terminal-to-`READY` gap, status-count summaries, and one coarse aggregate `cProfile` over executor callback work during measured runs only.
- The aggregate mirror profile is written only when `--profile-output` is non-empty.
- For raw arrival-order debugging without profiling logic, use `monitor_mirror_process.py` instead. That monitor is intentionally open-ended and does not auto-exit after `--runs`.
