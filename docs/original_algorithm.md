# Original DMAR Algorithm Specification

This document reconstructs the published algorithm in *Distributed Online
Rollout for Multivehicle Routing in Unmapped Environments* (Weber et al.,
AAMAS 2024, pp. 1910-1918). It is the specification for the CC-DMAR
reference implementation. Capacity constraints are an experiment-specific
extension and are not part of the published DMAR algorithm.

## 1. Scope and fidelity boundary

The paper defines DMAR for the UMVRP-L model. The implementation must keep
the following distinction:

- **Published mechanism:** directly described in Sections 2-3 or Algorithm 1.
- **Implementation assumption:** required because the paper leaves an
  executable detail unspecified, or because this project adds capacities.

The paper does not provide source code. It refers to Bertsekas [10] for the
full multiagent-rollout construction, so the exact internal rollout details
are not recoverable from this paper alone. Any such detail selected here is
labelled as an assumption in code and experiment reports.

## 2. Environment and agent state

**Paper, Section 2, p. 1911.** The known mathematical instance is a
rectangular 2D grid graph `G=(V,E)` with `N` nodes. `O` is a subset of
obstacle nodes that agents cannot occupy. `S` is a set of `m` uniquely
identified agents; co-location is allowed. `tau` is a subset of task nodes.
Time is discrete. At one time step an agent may move to an adjacent node. A
unit movement cost is charged for every agent that changes node. A task is
completed and removed when any agent visits its node.

The UMVRP-L setting makes `G`, `O`, initial agent positions, and task
locations unknown a priori. Agents have local memory and computation only.
At position `rho(a)`, agent `a` observes the subgraph induced by its current
node and all nodes within at most `k` graph hops, including obstacles,
neighboring agents, and tasks. The view is written `nu(a)`; the union over a
set of agents is `nu(S')`.

The distributed model is parallel and synchronous. All agents activate at
each time step and can use only information communicated at the preceding
time step. A computation may read the local view, including memories of
agents in it, and may modify memories of agents in that view.

### State retained by this implementation

Published state:

- unique immutable agent ID and current grid position;
- obstacle and traversability observations;
- discovered task locations and whether they remain unresolved;
- local memory used for communication, cluster pointers, flags, and maps;
- parent pointer, up to `c` child pointers, cluster ID, leader flag, join
  message, and map-message flag during a round.

Capacity extension (not in the paper):

- immutable initial budget `B[i]`;
- persistent remaining budget `remaining[i]`.

## 3. Local sensing and exploration

**Paper, Section 2 and Algorithm 1 lines 2 and 48-49, pp. 1911-1912.** A
view contains all nodes within `k` hops, so sensing is a graph-distance
ball, not necessarily a line-of-sight sensor. Agents not selected into a
cluster execute a non-empty random trajectory of length at most
`lambda(psi)`, selecting controls uniformly at random, and stop early when a
task enters their view. This random exploration is what makes DMAR applicable
to an unmapped environment and supports the expected `O(N^2)` completion-time
argument (Theorem 3.4, p. 1915).

**Assumptions required for code:** the random walk uses uniformly selected
legal adjacent moves, with waiting allowed only when no legal move exists;
the exact distribution of the length in `[1, lambda(psi)]` is not specified,
so the simulator uses a configurable deterministic-length or uniform-length
mode. The random generator and seed are explicit experiment parameters.

## 4. SOAC: self-organizing agent clusters

**Paper, Section 3, Algorithm 1 lines 2-28, Lemma 3.1, pp. 1912-1913.**
Agents know a common constant `psi`, which bounds cluster growth. A cluster is
represented by a rooted agent tree `T_K`; adjacent tree vertices are visible
to one another. The paper defines the maximum tree height as

`L(psi) = 3/2 * (2^psi - 2)`

and the child-pointer bound as a constant `c` (the typeset text gives the
resulting cluster size as `c^O(L(psi))`).

At round start, each agent clears its message flag, leader flag, cluster ID,
join message, children, and parent. It sets its leader flag if it sees a
task. A task-seeing agent clears that flag if it sees a task-seeing agent
with a larger ID. Every remaining leader sets its cluster ID to its own ID.

SOAC then repeats for `ceil(log2(psi))` synchronized iterations:

1. An unclustered agent that sees a clustered agent with a free child pointer
   appends itself as that agent's child and adopts its cluster ID.
2. Otherwise, if it sees agents from at most `c` distinct clusters, it may
   become a new leader and send join messages to those neighboring clusters.
   A recipient may reject a join. Join messages propagate for `L(psi)`
   synchronized iterations. A simultaneous recipient keeps one request;
   the paper says the choice is arbitrary.
3. On accepting a join, an agent adopts the requester's cluster ID, changes
   its parent, forwards the request to children and its former parent when
   applicable, then clears its child list and join message.

If a prospective parent has no free child pointer, the joining agent does not
join. The published algorithm does not impose a task count, capacity, or
pressure criterion on cluster recruitment.

**Deterministic implementation assumptions:** when several eligible parents,
join requests, or child orderings exist, choose by ascending agent ID; when
the paper says a choice is arbitrary, use the same deterministic rule. This
preserves reproducibility without claiming that tie rule is published.

## 5. LMA: local map aggregation

**Paper, Section 3, Algorithm 1 lines 29-42, Lemma 3.2, pp. 1913-1914.**
Every clustered agent starts with `M(a) = nu(a)`. The leader's local map is
labelled with its position as `(0,0)`. Map information is propagated down the
cluster tree and then back upward. Each union preserves the relative
coordinates by translating the received map through the parent/child
position offset. The leader therefore obtains

`M(K) = nu(K)`,

the union of the views of all agents in the cluster, expressed relative to
the leader. Agents outside the cluster are ignored even if they appear in a
view. The algorithm waits `L(psi)` synchronized iterations for each direction.

If the aggregate map has no usable tasks (for example, all such tasks are
unreachable), the paper says the cluster is dissolved by broadcast; the
precise dissolution message protocol is omitted from Algorithm 1.

**Assumptions required for code:** map cells use absolute simulator grid
coordinates internally, with leader-relative coordinates as the message
representation. Union is obstacle-consistent: an observed obstacle is never
made traversable. If conflicting observations occur, obstacle wins. The
paper does not specify a sensor-conflict policy.

## 6. Published base policy

**Paper, Sections 1.1 and 3, pp. 1911 and 1914.** The base policy used by
DMAR is a greedy nearest-neighbor heuristic: each agent moves toward its
respective nearest task in the assembled cluster map. DMAR's experiments
compare against a greedy-exploration policy that uses this same greedy
nearest-neighbor heuristic inside SOAC/LMA clusters but omits rollout.

The paper does not spell out whether nearest means Manhattan distance,
shortest obstacle-avoiding path, or another graph metric, nor how agents
choose between equal tasks or how multiple agents interact at a task. This
implementation uses shortest-path distance over the known aggregate grid,
then deterministic ties `(task coordinate, agent ID)`. A greedy base action
is one legal step along a shortest path to the selected nearest unresolved
task; it waits when no such path is known.

## 7. Sequential multiagent rollout

**Paper, Section 2, Equation (1), pp. 1911-1912; TMAR, Section 3, p. 1914.**
For state `x`, permissible joint controls `U(x)`, dynamics `f`, and
one-step movement cost `g`, rollout selects

`u_tilde in argmin_{u in U(x)} [g(x,u) + H_tilde(f(x,u))]`.

`H_tilde` is obtained by simulating a base policy. Direct joint enumeration
is exponential in the number of agents. Multiagent rollout reduces this by
selecting controls sequentially, one agent at a time:

1. Consider the candidate controls for the first agent in the fixed sequence.
2. For each candidate, fix that candidate, use already selected controls for
   earlier agents, and use base-policy controls for all later agents while
   estimating the resulting cost-to-go.
3. Select the candidate with the lowest immediate-plus-estimated cost.
4. Fix that agent's selected current action and continue to the next agent.
5. After the final agent is selected, execute the resulting vector of current
   controls and repeat on the next state.

In DMAR, the cluster leader performs this computation on `M(K)` and produces
one control sequence per cluster agent, then broadcasts the sequences down
`T_K`. TMAR is therefore rollout restricted to a temporary local team, not a
global centralized rollout.

**Important fidelity limit:** the paper delegates the detailed reformulation
and candidate-control construction to Bertsekas [10]. This document and the
project must not present a particular horizon, action enumeration, or
cost-to-go truncation as an original-paper detail. The simulator will expose
those as rollout configuration and report them as implementation assumptions.

The paper's rollout description says to fix controls selected for earlier
agents and use base-policy controls for later agents while evaluating a
candidate. It does not publish executable pseudocode stating whether those
later base-policy controls are recomputed from a state already updated by the
earlier selected controls. The implementation therefore retains its
documented assumption of evaluating later fallback actions from the round-start
state; changing this would be a rollout-semantic change, not a diagnostic
correction.

Capacity extension for CC-DMAR: rollout copies include copied remaining
capacities and unresolved-task state. A simulated task completion decrements
only the copied capacity and removes the task once. Real capacities change
only during execution of an actually completed task.

## 8. Execution, replanning, and termination

**Paper, Algorithm 1 lines 43-49 and Theorem 3.3, pp. 1912 and 1914-1915.**
The leader computes control sequences, broadcasts them for `L(psi)` steps,
and each clustered agent executes its assigned sequence. At the end of the
movement phase, agents unassign themselves from the cluster. Agents outside
clusters perform bounded random exploration. The phases repeat until all
tasks are completed. A round consists of SOAC, LMA, TMAR, and movement;
agents remain synchronized by waiting through fixed phase durations.

The paper gives `lambda(psi) = O(psi^4)` as a bound on the maximum route
length used for synchronization/random exploration, but does not give a
general executable formula beyond the asymptotic argument. It proves that a
round terminates in constant time for fixed `psi` and that expected completion
time is `O(N^2)` for solvable instances (Theorem 3.4). It also claims
probabilistic completeness (Theorem 3.5).

For this project, termination additionally occurs when a configured maximum
number of rounds or time steps is reached. A capacity-constrained run may
terminate with unresolved tasks when all agents have zero remaining capacity;
this is a new, explicit variant outcome rather than a property of published
DMAR.

## 9. Capacity variants and non-paper mechanisms

### CC-DMAR

CC-DMAR retains the published grid, sensing, exploration, SOAC, LMA, greedy
base policy, sequential rollout, execution, and replanning. It adds only:

- persistent per-agent initial budgets `B[i]` and remaining budgets;
- completion checks before a task is removed;
- copied budgets in rollout simulations;
- decrementing a real budget only when execution completes a task;
- allowing exhausted agents to explore and share maps but never complete tasks.

### CA-DMAR-RW

The proposed variant additionally uses the user-specified mechanisms:

- pressure-guided choice among otherwise eligible clusters,
  `pressure[K] = N_K / R_K` when `R_K > 0`, with zero-capacity clusters
  handled without division by zero;
- route insertion delta costs and capacity weighting
  `delta[i][j] * (R / remaining[i])^alpha`, default `alpha=0.5`;
- regret ordering, `second_lowest_delta - lowest_delta`, with explicit
  deterministic handling for zero or one eligible agents;
- provisional routes/capacities during assignment, never real capacities;
- the same sequential rollout architecture and movement-cost objective.

These additions are not described by Weber et al. and must be reported as a
new algorithm. Pressure must only choose among eligible clusters; it must not
replace movement cost as the rollout objective.

For experiments, exhausted-agent exploration is configurable without changing
the default CA-DMAR-RW behavior: random exploration is the default, stopping
movement is diagnostic-only, and deterministic nearest-frontier movement is an
experimental variant. Stopped exhausted agents still participate in SOAC and
LMA, so they remain available for communication and map aggregation.

## 10. Unrecoverable details to document in results

The paper does not fully specify: the exact multiagent-rollout algorithm from
[10], candidate action ordering, rollout horizon, tie-breaking, random-walk
length distribution, collision resolution for simultaneous moves, the exact
child-pointer constant `c`, the concrete `lambda(psi)` formula, map conflict
resolution, cluster dissolution messages, and whether tasks may be sensed
through obstacles. The implementation must expose or document each choice,
keep it deterministic where possible, and avoid calling it a reproduction of
an exact official implementation.

## 11. Proposed project structure

```text
docs/
  original_algorithm.md
  assumptions.md
include/dmar/
  environment.hpp       # grid, obstacles, tasks, shortest paths
  state.hpp              # agents, observations, capacities, task status
  communication.hpp      # synchronous views/messages
  clustering.hpp         # SOAC and cluster trees
  mapping.hpp            # LMA and coordinate-consistent maps
  rollout.hpp            # shared sequential rollout framework
  policies.hpp           # published greedy and proposed assignment policies
  variants.hpp           # CC-DMAR and CA-DMAR-RW configuration/dispatch
  metrics.hpp            # movement, completion, violations, timing
src/
  environment.cpp
  clustering.cpp
  mapping.cpp
  rollout.cpp
  policies.cpp
  simulator.cpp
  experiments.cpp
tests/
  test_capacity.cpp
  test_policies.cpp
  test_clustering.cpp
  test_rollout.cpp
  test_reproducibility.cpp
CMakeLists.txt
README.md
```

The implementation order is: shared grid/state and tests, published greedy
policy and rollout, SOAC/LMA, CC-DMAR capacity extension, then CA-DMAR-RW
heuristics and experiment comparison.