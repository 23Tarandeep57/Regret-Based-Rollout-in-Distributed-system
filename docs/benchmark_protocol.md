# Paper-comparable benchmark

`dmar_benchmark` follows the experimental protocol in Section 4 of the
paper:

- grid sides 10, 20, ..., 80;
- 20% of non-boundary cells designated as obstacles uniformly at random;
- agent count equal to the grid side;
- agent-to-task ratios 1:2, 1:1, and 2:1;
- uniform random task and agent positions;
- `psi = 8`;
- ten independently generated instances per grid size and ratio;
- ten runs per instance;
- sensing radius `k = 1..12` by default.

The same generated instance and random-walk seed are used for both
`GreedyExploration` and `CC_DMAR`. Instances for which a task is unreachable
from every initial agent are regenerated, matching the paper's exclusion of
geographically isolated tasks.

Build and run the complete benchmark:

```bash
cmake -S . -B build
cmake --build build
./build/dmar_benchmark > benchmark.csv
```

For a quick smoke test, the positional arguments are maximum grid side,
instances, runs, maximum rounds, and maximum sensing radius:

```bash
./build/dmar_benchmark 10 1 1 1000 3 0.5 > benchmark-small.csv
```

The seventh argument selects exhausted-agent exploration for CA-DMAR-R:
`0` enables the previous random-exploration behavior, `1` stops
exhausted agents while retaining them in clustering/map aggregation, and `2`
uses deterministic nearest-frontier exploration. The latter two are reported
as diagnostic/experimental variants; `1` is now the default for capacity
constrained benchmark runs.

The ninth argument selects the CA-DMAR-R SOAC policy:
`0` preserves the current capacity-aware behavior, `1` keeps exhausted agents
as communication/map members while excluding them from task planning, and `2`
uses original-style capacity-unaware SOAC membership with capacity still
enforced during assignment/completion. These are controlled experimental
comparisons; the default remains `0`.

For the controlled SOAC ablation, use modes:

- `0`: A, current capacity-aware SOAC with capacity-weighted assignment;
- `3`: B, all-agent SOAC with all cluster members entering rollout;
- `2`: C, all-agent SOAC with only eligible members entering rollout;
- `3` with assignment mode `1`: D, all-agent SOAC with greedy capacity assignment.

The tenth argument selects assignment mode: `0` is capacity-weighted and `1`
is greedy capacity-constrained assignment. Additional CSV fields report unique
clustered agents, cluster rounds, dissolution count, exhausted clustered-agent
rounds, exhausted-agent rounds, and task observations by exhausted agents.

The CSV includes movement cost, completed and unresolved tasks, rounds,
movement breakdowns, and observed traversable-cell coverage.
Aggregate by `algorithm`, `side`, `task_agent_ratio`, and `sensing_radius`
using the mean and a 95% confidence interval over instance/run rows.

Capacity-constrained benchmark runs assign a deterministic per-agent capacity
vector whose entries sum to the task count. The CSV additionally reports
`initial_capacity`, `remaining_capacity`, and `capacity_violations`. These
capacity results are an extension to the paper's unconstrained DMAR protocol.

This is a reproduction of the paper's stated instance protocol, not an exact
reproduction of its unpublished simulator. The repository's obstacle
generator, collision handling, rollout details, and stopping limit remain
implementation assumptions documented in `docs/original_algorithm.md`.
