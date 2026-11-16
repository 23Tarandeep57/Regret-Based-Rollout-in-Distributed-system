# Exploratory paths

This document records follow-up experiments motivated by the current benchmark
results. These are exploratory paths, not confirmed conclusions. Each path
should be run on the same generated instances and seeds where an ablation or
diagnostic comparison is intended.

## 1. Verify the idle-agent hypothesis

### Hypothesis

An unclustered agent with remaining capacity may stay still whenever any task
is inside its `view()`. If `view()` does not account for obstacles, a task
behind a wall could be visible but unreachable. The agent could then remain
idle permanently. This effect would become more likely at larger sensing
radii, potentially explaining the baseline's near-zero activity at large
radius and the three unsolvable instances. The same behavior may affect
CA-DMAR-R.

This is currently an unverified hypothesis based on reading the simulator, not
on direct diagnostic data.

### Instrumentation

Log the following per simulation round and algorithm:

- capable agents: agents with remaining capacity greater than zero;
- idle capable agents: capable agents that do not move or receive a task
  assignment during the round;
- tasks in view: tasks returned by each agent's `view()`;
- visible-but-unreachable tasks: tasks in view for which no traversable path
  exists from the observing agent's current position;
- movement reason: task execution, exploration, or no movement;
- agent cluster state and remaining capacity.

Aggregate both counts and agent-round rates. A useful diagnostic row is:

```text
algorithm,side,sensing_radius,round,idle_capable_agents,
visible_tasks,visible_unreachable_tasks
```

Compare idle-capable rates and visible-but-unreachable rates across sensing
radii, map sizes, and algorithms. A strong confirmation would show that idle
capable agents frequently have visible-but-unreachable tasks and that this
rate increases with sensing radius.

Also verify the exhausted-agent counters before interpreting them. Rename
`rounds_with_clusters` to a name that states what is counted (for example,
`clustered_agent_rounds` or `rounds_with_any_cluster`) and document whether
the unit is a round, an agent-round, or a cluster.

## 2. Make capacity pressure observable

Extend the benchmark with heterogeneous capacities and slack:

- assign different capacities to agents;
- include cases where total capacity is greater than the number of tasks;
- retain a deterministic capacity vector per generated instance;
- record capacity distribution, total initial capacity, and unused capacity.

Run the full alpha boundary:

- `alpha = 0`: no capacity-pressure preference;
- the current intermediate alpha values;
- `alpha = 1`: maximum capacity-pressure preference.

Measure completion, unresolved tasks, rejected assignments, movement, rounds,
capacity violations, and the capacity of the selected agent at assignment
time. The capacity-weighted score and pressure rule should only be evaluated
as meaningful if these scenarios contain both heterogeneous remaining
capacities and enough slack for assignments to be selective.

## 3. Run one-modification-at-a-time ablations

Use the current CA-DMAR-R configuration as the control and enable exactly one
modification per treatment. At minimum, isolate:

1. exhausted-agent handling;
2. capacity-aware SOAC membership;
3. capacity-weighted assignment;
4. frontier exploration;
5. any revised visibility or reachability handling introduced while testing
   the idle-agent hypothesis.

Keep the instance, run seed, map, task count, and sensing radius fixed within
each ablation. Report paired deltas against the control rather than comparing
unpaired aggregate means. Include the same movement, completion, timeout,
capacity, and diagnostics fields for every variant.

## 4. Expand the benchmark and reduce censoring

Add more independently generated instances before drawing conclusions from
the three currently unsolvable cases. Preserve the existing reachability
screening for the paper-comparable benchmark, but report separately whether an
instance was rejected, regenerated, or became unsolvable during simulation.

For large maps with small sensing radii, raise the maximum round cap or run a
separate exploration-limited protocol. Mark the protocol in output metadata
so capped runs are not mixed with the standard benchmark. Report:

- completion and unresolved-task rates;
- timeout/censoring rate;
- median and upper-quantile rounds;
- movement cost and movement by reason;
- observed traversable-cell coverage;
- idle-capable and visible-but-unreachable rates.

The primary comparison should use the same cap across algorithms. A
secondary sensitivity analysis can vary the cap to distinguish genuine
algorithmic failure from insufficient exploration time.

## Suggested execution order

1. Correct and document counter semantics.
2. Add idle/reachability instrumentation and inspect a small deterministic
   sample.
3. Run the capacity/slack and alpha-boundary matrix.
4. Run the one-modification-at-a-time ablations.
5. Expand instances and repeat the large-map, small-radius cases with a
   higher round cap.

Do not treat an observed correlation as confirmation of the idle-agent
hypothesis until the per-round logs establish the relationship between
visibility, reachability, and inactivity.
