Proposed solution details - https://docs.google.com/document/d/1H9iBHOUo-DxjXVHIbuHGcFhOiLCdhyu_p5ek8Bvi4eQ/edit?usp=sharing

This work is extention of the problem posed in the paper : https://arxiv.org/abs/2305.15596

This repository contains a benchmark for Greedy exploration, DMAR(original paper), and
capacity-aware CA-DMAR-R (The proposed solution).

Requirements

- CMake 3.16 or newer
- A C++17 compiler

Run the complete experiment

From the repository root:

    chmod +x run_benchmark.sh
    ./run_benchmark.sh

The script builds dmar_benchmark and writes these nine files:

    benchmark-10x10-alpha-0.25.csv
    benchmark-10x10-alpha-0.5.csv
    benchmark-10x10-alpha-0.75.csv
    benchmark-20x20-alpha-0.25.csv
    benchmark-20x20-alpha-0.5.csv
    benchmark-20x20-alpha-0.75.csv
    benchmark-30x30-alpha-0.25.csv
    benchmark-30x30-alpha-0.5.csv
    benchmark-30x30-alpha-0.75.csv

Each command uses:

- 5 generated instances
- 3 runs per instance
- map sides from 10 to the requested maximum side
- sensing radius 1 through 8
- 5,000 simulation steps maximum
- alpha values 0.25, 0.5, and 0.75
- exhausted-agent mode 1
- current SOAC policy and capacity-weighted assignment

Mode definitions

The exhausted-agent mode controls what an agent does after its remaining
capacity reaches zero:

- mode 0: the exhausted agent continues random exploration
- mode 1: the exhausted agent stops moving, but still participates in
  clustering and map aggregation
- mode 2: the exhausted agent uses frontier exploration

The benchmark script uses exhausted-agent mode 1. This enforces the capacity
constraint: an exhausted agent is not dispatched for task completion or task
movement, but remains available to contribute observations and map
aggregation.

The SOAC mode controls cluster membership behavior:

- mode 0: current capacity-aware SOAC policy
- mode 1: communication-only membership for exhausted agents
- mode 2: original-style membership with eligible agents planning tasks
- mode 3: original-style membership with all cluster members planning

The assignment mode controls task assignment:

- mode 0: capacity-weighted assignment
- mode 1: greedy capacity-constrained assignment

The benchmark script uses SOAC mode 0 and assignment mode 0.

The script runs the benchmark separately for maximum sides 10, 20, and 30.
Therefore each output file contains only its named map size. The executable
prints progress to stderr and CSV data to stdout.

Optional environment variables

The defaults can be changed without editing the script:

    INSTANCES=10 RUNS=5 MAX_ROUNDS=5000 MAX_SENSING_RADIUS=8 ./run_benchmark.sh

The benchmark can take a long time, especially for 30x30 maps.

Meaning of rounds

One round is one actual simulation step. During a round the simulator forms
clusters, aggregates local maps, assigns eligible agents, computes controls,
moves the agents, and completes tasks at their new positions.

The rollout horizon contains hypothetical future simulations. Those
hypothetical steps do not increment the reported rounds.

Important CSV fields

- movement_cost: total agent-cell movements during a run
- rounds: actual simulation steps used
- completed_tasks: tasks completed before stopping
- unresolved_tasks: tasks remaining at the end
- rejected_assignments: assignment requests rejected by the capacity-aware
  policy
- remaining_capacity: total task capacity remaining at the end
- exhausted_movement: movement made by exhausted agents
- exhausted_clustered_movement: movement made by exhausted agents while
  clustered
- tasks_seen_by_exhausted_agents: tasks observed by exhausted agents

An agent with zero remaining capacity can still participate in clustering and
map aggregation. It must not be dispatched for task completion or movement.

Current results

The EDA notebook (`benchmark_eda.ipynb`) analyzes the nine benchmark files:
9,720 rows, 45 independent map layouts (15 per map side), eight sensing
radii, three runs per scenario, and a 5,000-round cap. Confidence intervals
in the notebook are bootstrapped over the 45 layouts rather than over
individual rows, because radii and runs reuse each layout.

The following results are instance-level means. Movement is total movement per
run, not movement per round.

All-map aggregate:

    algorithm    completion    mean unresolved tasks    mean rounds
    greedy       58.3 percent  1.132                   2864
    dmar         60.0 percent  1.010                   2820
    ca_dmar_r    80.6 percent  0.371                   2005

CA-DMAR-R improves completion over DMAR by 20.6 percentage points
(95% bootstrap CI: 16.2 to 25.3 points), and is better on 39 of 45 layouts
(tied on four and worse on two). On matched scenarios, 27% finish only with
CA-DMAR-R, while 6.5% finish only with DMAR.

The completion gain does not imply lower total cost. Among scenarios where
both algorithms finish, CA-DMAR-R uses 1.17 times DMAR's total movement
(95% CI: 1.08 to 1.27) and 1.14 times its rounds (1.03 to 1.27). Its
task-serving movement is lower, at 0.93 times DMAR's (0.91 to 0.96), but
96-97% of all movement is exploration.

CA-DMAR-R by alpha

    alpha    completion
    0.25     80.3 percent
    0.50     80.5 percent
    0.75     81.1 percent

Interpretation

CA-DMAR-R completes substantially more runs, but it generally uses more total
movement when both algorithms finish. The data is consistent with a reduction
in stalling rather than a demonstrated benefit from capacity weighting.

Baseline failure rates increase with sensing radius: DMAR rises from 27% at
radius 1 to 69% at radius 8, while CA-DMAR-R falls from 30% to 12%. On 10x10
maps at radius 8, the corresponding rates are 87% for DMAR and 7% for
CA-DMAR-R. At radius 6 and above, failed baseline runs contain only about 1%
moving agent-rounds, compared with 3-4% for CA-DMAR-R.

There are two failure regimes: large maps with small radii are
exploration-limited by the 5,000-round cap, while large radii produce
stall-limited baseline failures. Three layouts fail for every algorithm in
every scenario, so the observed completion ceiling is approximately 93%.

Alpha has little detectable effect. The benchmark gives every agent the same
capacity (0.5, 1, or 2 tasks per agent), so `(remaining capacity / agent
capacity)^alpha` is identical across eligible agents. Results are identical
across alpha in all 0.5- and 1-task-per-agent scenarios; only 2-tasks-per-agent
scenarios differ. This benchmark therefore does not test the capacity-weighted
score.

Capacity and measurement caveats

Across the analyzed benchmark files:

- capacity violations were zero
- exhausted-agent movement was zero
- exhausted clustered-agent movement was zero
- exhausted agents were counted as cluster members in 99% of rows
- exhausted-agent rounds were present in 100% of rows

The exhausted-agent counters are internally inconsistent: exhausted agents are
counted as cluster members, but `exhausted_movement` and
`exhausted_clustered_rounds` are zero in every row while
`exhausted_exploration` is set everywhere. Verify these counters in the
simulator before using them.

The benchmark has zero capacity slack: total capacity equals task count, so
each unresolved task is also one unused capacity unit. Unfinished runs are
right-censored at 5,000 rounds; their movement and round counts are lower
bounds. Compare cost and rounds conditionally on both algorithms finishing.
`rounds_with_clusters` is a cluster-round count rather than a round count and
is misleadingly named. `rejected_assignments` is heavy-tailed and likely
includes repeated per-round rejections.

The notebook data uses constant `soac_mode`, `assignment_mode`, and
`exhausted_exploration`, so it contains no ablation evidence for the
individual CA-DMAR-R modifications. See
`docs/exploratory_paths.md` for proposed diagnostics, heterogeneous-capacity
experiments, ablations, and higher-cap round-cap runs.

Use completion rate, unresolved tasks, censoring rate, and conditional
movement/rounds together. Uncensored movement and rounds are lower bounds for
failed runs, and means are strongly affected by timeout outliers.
