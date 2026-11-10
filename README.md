DMAR benchmark reproduction

This repository contains a benchmark for Greedy exploration, DMAR, and
capacity-aware CA-DMAR-R.

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

The following results summarize the benchmark CSV files generated in this
repository. Movement is total movement per run, not movement per round.

All-map aggregate:

    algorithm    completion    median movement    median rounds    timeout rate
    greedy       60.1 percent  485                1964             39.9 percent
    dmar         62.2 percent  456                1930             37.8 percent
    ca_dmar_r    85.0 percent  688                575              15.0 percent

Results by map size:

    map      algorithm    completion    median movement    median rounds
    10x10    greedy       61.7 percent  77.5               395
    10x10    dmar         65.0 percent  74                 283
    10x10    ca_dmar_r    92.2 percent  106.5              90

    20x20    greedy       62.2 percent  1604               2002.5
    20x20    dmar         63.1 percent  1350               2456.5
    20x20    ca_dmar_r    84.0 percent  2088               1307

    30x30    greedy       51.1 percent  5505               4925
    30x30    dmar         51.9 percent  5588.5             4684
    30x30    ca_dmar_r    65.6 percent  6329               3757.5

CA-DMAR-R by alpha:

    map      alpha    completion    median movement    median rounds
    10x10    0.25     92.2 percent  102.5              91.5
    10x10    0.50     92.2 percent  101.5              89
    10x10    0.75     92.2 percent  110                89.5

    20x20    0.25     83.6 percent  2109               1333.5
    20x20    0.50     84.2 percent  2058               1294.5
    20x20    0.75     84.2 percent  2082.5             1301

    30x30    0.25     65.0 percent  6513               3806.5
    30x30    0.50     65.0 percent  6429               3757.5
    30x30    0.75     66.9 percent  6060               3701

Interpretation

CA-DMAR-R completes substantially more runs and reaches completion in fewer
rounds, but it generally uses more total movement. Greedy and DMAR have lower
typical movement but hit the 5,000-step limit more often.

Performance degrades as the map grows. The advantage of CA-DMAR-R remains
visible on 30x30 maps, although all algorithms have many timeout runs.

Alpha has little effect on Greedy and DMAR. Alpha 0.5 is a reasonable general
default for CA-DMAR-R. Alpha 0.75 is slightly better in the current 30x30
results.

Capacity validation

Across the completed benchmark files:

- capacity violations were zero
- exhausted-agent movement was zero
- exhausted clustered-agent movement was zero
- exhausted agents still observed tasks in most runs

Use completion rate, timeout rate, median movement, median rounds, and
unresolved tasks together. Mean movement is strongly affected by timeout
outliers.
