#include "dmar/simulator.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>

namespace dmar {

namespace {

bool task_in_view(
    const SimulationState& state, const Agent& agent) {
    for (const Position cell : state.environment().view(agent.position, agent.sensing_radius)) {
        if (state.environment().has_task(cell)) return true;
    }
    return false;
}

Control greedy_control(
    const SimulationState& state, std::size_t index, const std::vector<Position>& known_tasks,
    const std::vector<Position>& assigned_tasks, bool capacity_aware) {
    const Agent& agent = state.agents()[index];
    // Simulated capacities change inside rollout copies; an agent whose simulated budget is
    // spent must stop chasing tasks it can no longer complete.
    if (capacity_aware && !agent.can_perform_task()) return agent.position;
    std::optional<int> best_distance;
    Position best = agent.position;
    std::vector<Position> candidates;
    const auto append_unresolved = [&](const std::vector<Position>& source) {
        for (const Position task : source) {
            if (state.environment().has_task(task)) candidates.push_back(task);
        }
    };
    append_unresolved(assigned_tasks);
    if (candidates.empty()) append_unresolved(known_tasks);
    for (const Position task : candidates) {
        const auto distance = state.environment().shortest_distance(agent.position, task);
        if (distance.has_value() && (!best_distance.has_value() || *distance < *best_distance)) {
            best_distance = distance;
            best = task;
        }
    }
    if (best == agent.position) return best;
    const auto step = state.environment().shortest_step(agent.position, best);
    return step.value_or(agent.position);
}

Control frontier_step_impl(const SimulationState& state, std::size_t agent_index) {
    const Agent& agent = state.agents()[agent_index];
    std::optional<int> best_distance;
    Position best = agent.position;
    for (int row = 0; row < state.environment().rows(); ++row) {
        for (int column = 0; column < state.environment().columns(); ++column) {
            const Position frontier{row, column};
            if (state.observed(frontier) || !state.environment().traversable(frontier)) continue;
            bool adjacent_to_observed = false;
            for (const Position neighbor : state.environment().legal_moves(frontier)) {
                if (state.observed(neighbor)) {
                    adjacent_to_observed = true;
                    break;
                }
            }
            if (!adjacent_to_observed) continue;
            const auto distance = state.environment().shortest_distance(agent.position, frontier);
            if (!distance.has_value()) continue;
            if (!best_distance.has_value() || *distance < *best_distance ||
                (*distance == *best_distance &&
                 (frontier.row < best.row ||
                  (frontier.row == best.row && frontier.column < best.column)))) {
                best_distance = distance;
                best = frontier;
            }
        }
    }
    if (best == agent.position) return best;
    return state.environment().shortest_step(agent.position, best).value_or(agent.position);
}

double step_cost(const SimulationState& before, const SimulationState& after) {
    double cost = 0.0;
    for (std::size_t index = 0; index < before.agents().size(); ++index) {
        if (before.agents()[index].position != after.agents()[index].position) cost += 1.0;
    }
    return cost;
}

}  // namespace

Control deterministic_frontier_step(const SimulationState& state, std::size_t agent_index) {
    return frontier_step_impl(state, agent_index);
}

Metrics run_simulation(SimulationState state, const SimulatorConfig& config) {
    Metrics metrics;
    std::mt19937 random(config.seed);
    const bool capacity_aware = config.algorithm == Algorithm::CA_DMAR_RW;
    const bool capacity_aware_soac =
        capacity_aware &&
        config.soac_policy != SoacPolicy::OriginalStyleEligible &&
        config.soac_policy != SoacPolicy::OriginalStyleAllMembers;
    std::vector<bool> ever_clustered(state.agents().size(), false);
    bool had_clusters = false;
    const std::size_t agent_count = state.agents().size();
    for (const Agent& agent : state.agents()) state.observe(agent);
    metrics.total_traversable_cells = 0;
    for (int row = 0; row < state.environment().rows(); ++row)
        for (int column = 0; column < state.environment().columns(); ++column)
            if (state.environment().traversable({row, column})) ++metrics.total_traversable_cells;
    metrics.completed_tasks_by_agent.assign(agent_count, 0);
    metrics.movement_by_agent.assign(agent_count, 0);
    metrics.eligible_movement_by_agent.assign(agent_count, 0);
    metrics.exhausted_movement_by_agent.assign(agent_count, 0);
    metrics.exhausted_clustered_movement_by_agent.assign(agent_count, 0);
    metrics.exhausted_unclustered_movement_by_agent.assign(agent_count, 0);
    metrics.task_serving_movement_by_agent.assign(agent_count, 0);
    metrics.exploration_movement_by_agent.assign(agent_count, 0);
    const auto total_remaining_capacity = [](const SimulationState& current) {
        int total = 0;
        for (const Agent& agent : current.agents()) total += std::max(0, agent.remaining_capacity);
        return total;
    };
    for (int round = 0; round < config.max_rounds && state.environment().unresolved_tasks() > 0; ++round) {
        // Capacity exhaustion is a termination condition: nobody can complete another task.
        if (capacity_aware && total_remaining_capacity(state) == 0) break;
        std::vector<Cluster> clusters = form_soac_clusters(
            state, config.psi, config.child_limit, capacity_aware_soac);
        if (had_clusters && clusters.empty()) ++metrics.cluster_dissolutions;
        had_clusters = !clusters.empty();
        aggregate_local_maps(state, clusters);
        std::vector<bool> clustered(state.agents().size(), false);
        std::vector<int> member_cluster_ids(state.agents().size(), 0);
        std::vector<bool> map_contributors(state.agents().size(), false);
        for (const Cluster& cluster : clusters) {
            ++metrics.rounds_with_clusters;
            metrics.max_cluster_size = std::max(metrics.max_cluster_size, cluster.agent_indices.size());
            for (const std::size_t member : cluster.agent_indices) {
                ++metrics.cluster_memberships;
                ever_clustered[member] = true;
                if (!state.agents()[member].can_perform_task())
                    ++metrics.exhausted_cluster_memberships;
            }
            for (const std::size_t index : cluster.agent_indices) {
                member_cluster_ids[index] = cluster.leader_id;
                map_contributors[index] = true;
            }
        }
        std::vector<Control> round_controls(state.agents().size());
        for (std::size_t index = 0; index < state.agents().size(); ++index) {
            round_controls[index] = state.agents()[index].position;
        }
        for (const Cluster& cluster : clusters) {
            std::vector<std::size_t> order = cluster.agent_indices;
            const bool communication_membership =
                config.soac_policy == SoacPolicy::CommunicationOnly ||
                config.soac_policy == SoacPolicy::OriginalStyleEligible ||
                config.soac_policy == SoacPolicy::OriginalStyleAllMembers;
            if (communication_membership) {
                for (const std::size_t index : cluster.agent_indices) clustered[index] = true;
            }
            std::sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
                return state.agents()[left].id < state.agents()[right].id;
            });
            if (config.soac_policy != SoacPolicy::OriginalStyleAllMembers) {
                order.erase(std::remove_if(order.begin(), order.end(), [&](std::size_t index) {
                    return !state.agents()[index].can_perform_task();
                }), order.end());
            }
            std::vector<Position> known_tasks;
            for (const auto& cell : cluster.map_cells) {
                if (cell.second && state.environment().has_task(cell.first)) known_tasks.push_back(cell.first);
            }
            if (order.empty()) continue;
            std::vector<Position> reachable_tasks;
            for (const Position task : known_tasks) {
                for (const std::size_t index : order) {
                    if (state.environment().shortest_distance(
                            state.agents()[index].position, task).has_value()) {
                        reachable_tasks.push_back(task);
                        break;
                    }
                }
            }
            known_tasks = std::move(reachable_tasks);
            if (known_tasks.empty()) continue;
            for (const std::size_t index : order) {
                clustered[index] = true;
            }

            std::vector<std::vector<Position>> assigned_tasks(state.agents().size());
            if (config.algorithm == Algorithm::CA_DMAR_RW &&
                config.assignment_policy == AssignmentPolicy::CapacityWeighted) {
                std::vector<Position> tasks = known_tasks;
                const std::vector<Assignment> assignments = capacity_weighted_assign(
                    state.environment(), state.agents(), order, tasks, config.alpha);
                metrics.rejected_assignments += tasks.size() - assignments.size();
                for (const Assignment& assignment : assignments) {
                    assigned_tasks[assignment.agent_index].push_back(assignment.task);
                    if (config.debug_logging) {
                        std::cerr << "  assignment agent=" << state.agents()[assignment.agent_index].id
                                  << " task=(" << assignment.task.row << ',' << assignment.task.column
                                  << ") remaining-before="
                                  << state.agents()[assignment.agent_index].remaining_capacity
                                  << ") delta=" << assignment.unweighted_delta
                                  << " regret=" << assignment.regret
                                  << " weighted=" << assignment.weighted_cost << '\n';
                        std::cerr << "    route-before=";
                        for (const Position task : assignment.route_before)
                            std::cerr << '(' << task.row << ',' << task.column << ") ";
                        std::cerr << " route-after=";
                        for (const Position task : assignment.route_after)
                            std::cerr << '(' << task.row << ',' << task.column << ") ";
                        std::cerr << '\n';
                    }
                }
            } else if (config.algorithm == Algorithm::CA_DMAR_RW) {
                for (const std::size_t index : order) {
                    if (!state.agents()[index].can_perform_task()) continue;
                    for (const Position task : known_tasks) {
                        if (state.environment().shortest_distance(
                                state.agents()[index].position, task).has_value()) {
                            assigned_tasks[index].push_back(task);
                            break;
                        }
                    }
                }
            }
            const BasePolicy base_policy = [&](const SimulationState& current, std::size_t index) {
                return greedy_control(
                    current, index, known_tasks, assigned_tasks[index], capacity_aware);
            };
            if (config.algorithm == Algorithm::GreedyExploration) {
                for (const std::size_t index : order) {
                    round_controls[index] = base_policy(state, index);
                }
            } else {
                RolloutConfig rollout_config;
                rollout_config.known_tasks = known_tasks;
                rollout_config.horizon = state.environment().rows() + state.environment().columns();
                const RolloutResult rollout = sequential_rollout(
                    state, order, base_policy, rollout_config);
                bool moved = false;
                for (const std::size_t index : order) {
                    if (rollout.controls[index] != state.agents()[index].position) moved = true;
                }
                for (const std::size_t index : order) {
                    round_controls[index] = moved
                        ? rollout.controls[index]
                        : base_policy(state, index);
                }
            }
        }
        for (std::size_t index = 0; index < state.agents().size(); ++index) {
            if (!state.agents()[index].can_perform_task()) {
                ++metrics.exhausted_agent_rounds;
                if (clustered[index]) ++metrics.exhausted_clustered_rounds;
                for (const Position cell : state.environment().view(
                         state.agents()[index].position, state.agents()[index].sensing_radius)) {
                    if (state.environment().has_task(cell)) {
                        ++metrics.tasks_seen_by_exhausted_agents;
                        break;
                    }
                }
            }
            if (clustered[index]) continue;
            const bool exhausted = !state.agents()[index].can_perform_task();
            // Baseline: exhausted agents idle. CA-DMAR-R: they keep exploring (random walk)
            // so they still contribute sensing and relaying; they never wait on a task since
            // they cannot serve it.
            if (exhausted && (!capacity_aware ||
                              config.exhausted_exploration == ExhaustedExploration::Stop)) continue;
            Position next = state.agents()[index].position;
            if (exhausted && config.exhausted_exploration == ExhaustedExploration::Frontier) {
                next = deterministic_frontier_step(state, index);
            } else if (exhausted || !task_in_view(state, state.agents()[index])) {
                next = state.environment().random_legal_move(next, random);
            }
            round_controls[index] = next;
        }
        if (config.debug_logging) {
            std::size_t reachable_unresolved = 0;
            for (int row = 0; row < state.environment().rows(); ++row) {
                for (int column = 0; column < state.environment().columns(); ++column) {
                    const Position task{row, column};
                    if (!state.environment().has_task(task)) continue;
                    bool reachable = false;
                    for (const Agent& agent : state.agents()) {
                        if (agent.can_perform_task() &&
                            state.environment().shortest_distance(agent.position, task).has_value()) {
                            reachable = true;
                            break;
                        }
                    }
                    if (reachable) ++reachable_unresolved;
                }
            }
            std::cerr << "round " << (round + 1) << " clusters=" << clusters.size()
                      << " unresolved=" << state.environment().unresolved_tasks()
                      << " eligible_reachable_unresolved=" << reachable_unresolved << '\n';
            for (const Cluster& cluster : clusters) {
                std::cerr << "  cluster leader=" << cluster.leader_id << " members=";
                for (const std::size_t index : cluster.agent_indices)
                    std::cerr << state.agents()[index].id << ' ';
                std::cerr << '\n';
            }
            for (std::size_t index = 0; index < agent_count; ++index) {
                const Agent& agent = state.agents()[index];
                const bool exhausted = !agent.can_perform_task();
                const bool sees_task = task_in_view(state, agent);
                const bool planning_member = clustered[index];
                const bool actual_member = map_contributors[index];
                const bool moved = round_controls[index] != agent.position;
                const char* branch = planning_member
                    ? (exhausted ? "clustered-exhausted-hold" : "cluster-rollout")
                    : (exhausted
                           ? (config.exhausted_exploration == ExhaustedExploration::Frontier
                                  ? "unclustered-exhausted-frontier"
                                  : (config.exhausted_exploration == ExhaustedExploration::Stop
                                         ? "unclustered-exhausted-stop"
                                         : "unclustered-exhausted-random"))
                           : "unclustered-capable-random");
                const bool random_branch =
                    !planning_member && (std::string(branch) == "unclustered-exhausted-random" ||
                                         std::string(branch) == "unclustered-capable-random");
                std::cerr << "  agent=" << agent.id << " pos=(" << agent.position.row << ','
                          << agent.position.column << ") remaining=" << agent.remaining_capacity
                          << " cluster_id=" << member_cluster_ids[index]
                          << " map_member=" << actual_member
                          << " planning_member=" << planning_member
                          << " task_visible=" << sees_task
                          << " branch=" << branch
                          << " random_steps=" << (random_branch && moved ? 1 : 0)
                          << " control=(" << round_controls[index].row << ','
                          << round_controls[index].column << ")\n";
            }
        }
        const SimulationState before = state;
        state = apply_controls(state, round_controls);
        for (const Agent& agent : state.agents()) state.observe(agent);
        metrics.movement_cost += step_cost(before, state);
        metrics.rounds = round + 1;
        if (config.debug_logging) {
            std::cerr << "  task-progress unresolved-before=" << before.environment().unresolved_tasks()
                      << " unresolved-after=" << state.environment().unresolved_tasks()
                      << " completed-this-round="
                      << (before.environment().unresolved_tasks() -
                          state.environment().unresolved_tasks()) << '\n';
        }
        for (std::size_t index = 0; index < agent_count; ++index) {
            const Agent& old_agent = before.agents()[index];
            const Agent& new_agent = state.agents()[index];
            const bool moved = old_agent.position != new_agent.position;
            const bool task_serving = clustered[index] && old_agent.can_perform_task();
            if (moved) {
                ++metrics.movement_by_agent[index];
                if (old_agent.can_perform_task()) ++metrics.eligible_movement_by_agent[index];
                else {
                    ++metrics.exhausted_movement_by_agent[index];
                    if (clustered[index]) ++metrics.exhausted_clustered_movement_by_agent[index];
                    else ++metrics.exhausted_unclustered_movement_by_agent[index];
                }
                if (task_serving) ++metrics.task_serving_movement_by_agent[index];
                else ++metrics.exploration_movement_by_agent[index];
            }
            metrics.observed_traversable_cells = state.observed_traversable_cells();
            if (old_agent.can_perform_task() &&
                before.environment().has_task(new_agent.position) &&
                !state.environment().has_task(new_agent.position)) {
                ++metrics.completed_tasks_by_agent[index];
            }
            if (config.debug_logging) {
                std::cerr << "  result agent=" << new_agent.id << " pos=(" << new_agent.position.row
                          << ',' << new_agent.position.column << ") remaining="
                          << new_agent.remaining_capacity << " moved=" << moved
                          << " mode=" << (task_serving ? "task-serving" : "exploration")
                          << " cumulative-movement=" << metrics.movement_by_agent[index] << '\n';
            }
        }
    }
    metrics.completed_tasks = state.completed_tasks();
    metrics.unresolved_tasks = state.environment().unresolved_tasks();
    metrics.capacity_violations = state.capacity_violations();
    for (const bool value : ever_clustered) {
        if (value) ++metrics.unique_clustered_agents;
    }
    metrics.execution_capacity_violations = state.capacity_violations();
    for (const Agent& agent : state.agents()) {
        metrics.initial_capacity += agent.initial_capacity;
        metrics.remaining_capacity += agent.remaining_capacity;
    }
    return metrics;
}

}  // namespace dmar