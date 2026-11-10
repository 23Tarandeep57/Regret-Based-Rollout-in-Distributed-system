#include "dmar/clustering.hpp"
#include "dmar/policies.hpp"

#include <algorithm>
#include <cstdlib>
#include <map>
#include <set>
#include <utility>

namespace dmar {

namespace {

int distance(Position left, Position right) {
    return std::abs(left.row - right.row) + std::abs(left.column - right.column);
}

bool sees_task(const SimulationState& state, const Agent& agent) {
    for (const Position cell : state.environment().view(agent.position, agent.sensing_radius)) {
        if (state.environment().has_task(cell)) return true;
    }
    return false;
}

struct ClusterLoad {
    std::size_t known_unresolved = 0;
    int residual_capacity = 0;
    double pressure = 0.0;
};

// N_K counts each unresolved task once, even if several member views overlap on it.
ClusterLoad cluster_load(const SimulationState& state, const std::vector<std::size_t>& members) {
    std::set<std::pair<int, int>> tasks;
    ClusterLoad load;
    for (const std::size_t member : members) {
        const Agent& agent = state.agents()[member];
        load.residual_capacity += std::max(0, agent.remaining_capacity);
        for (const Position cell : state.environment().view(agent.position, agent.sensing_radius)) {
            if (state.environment().has_task(cell)) tasks.emplace(cell.row, cell.column);
        }
    }
    load.known_unresolved = tasks.size();
    load.pressure = cluster_pressure(load.known_unresolved, load.residual_capacity);
    return load;
}

}  // namespace

std::vector<Cluster> form_soac_clusters(
    const SimulationState& state, int psi, int child_limit, bool capacity_aware) {
    if (psi <= 0 || child_limit <= 0) return {};

    // Leader election. In capacity-aware mode an exhausted agent can neither be a leader nor
    // disqualify a neighbour from leading (it can still be recruited below).
    const auto eligible_leader = [&](const Agent& agent) {
        if (capacity_aware && !agent.can_perform_task()) return false;
        return sees_task(state, agent);
    };
    std::vector<std::size_t> leaders;
    for (std::size_t index = 0; index < state.agents().size(); ++index) {
        if (!eligible_leader(state.agents()[index])) continue;
        bool disqualified = false;
        for (std::size_t other = 0; other < state.agents().size(); ++other) {
            if (other == index || !eligible_leader(state.agents()[other])) continue;
            if (distance(state.agents()[index].position, state.agents()[other].position) <=
                    state.agents()[index].communication_radius &&
                state.agents()[other].id > state.agents()[index].id) {
                disqualified = true;
            }
        }
        if (!disqualified) leaders.push_back(index);
    }
    std::sort(leaders.begin(), leaders.end(), [&](std::size_t left, std::size_t right) {
        return state.agents()[left].id < state.agents()[right].id;
    });
    std::vector<Cluster> clusters;
    for (const std::size_t leader : leaders) clusters.push_back(Cluster{state.agents()[leader].id, {leader}, {}});
    std::vector<bool> assigned(state.agents().size(), false);
    for (const Cluster& cluster : clusters) assigned[cluster.agent_indices.front()] = true;
    for (int iteration = 0; iteration < psi; ++iteration) {
        for (std::size_t agent_index = 0; agent_index < state.agents().size(); ++agent_index) {
            if (assigned[agent_index]) continue;
            std::size_t best_cluster = clusters.size();
            double best_pressure = -1.0;
            for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
                const Cluster& cluster = clusters[cluster_index];
                if (static_cast<int>(cluster.agent_indices.size()) >= child_limit) continue;
                const bool joinable = std::any_of(
                    cluster.agent_indices.begin(), cluster.agent_indices.end(),
                    [&](std::size_t member) {
                        return distance(state.agents()[agent_index].position,
                                        state.agents()[member].position) <=
                            state.agents()[agent_index].communication_radius;
                    });
                if (!joinable) continue;
                const double pressure =
                    capacity_aware ? cluster_load(state, cluster.agent_indices).pressure : 0.0;
                // Higher pressure wins; equal pressure (always, in baseline mode) falls back
                // to the lowest leader ID.
                if (best_cluster == clusters.size() || pressure > best_pressure ||
                    (pressure == best_pressure &&
                     cluster.leader_id < clusters[best_cluster].leader_id)) {
                    best_cluster = cluster_index;
                    best_pressure = pressure;
                }
            }
            if (best_cluster != clusters.size()) {
                clusters[best_cluster].agent_indices.push_back(agent_index);
                assigned[agent_index] = true;
            }
        }
    }

    // Deterministic approximation of Algorithm 1's cluster-join phase:
    // an unclustered agent that sees multiple clusters roots their merge.
    // The merged cluster keeps the lowest-ID existing leader, so in capacity-aware mode the
    // merged cluster is still rooted at an agent with r_i > 0.
    for (std::size_t agent_index = 0; agent_index < state.agents().size(); ++agent_index) {
        if (assigned[agent_index]) continue;
        std::vector<std::size_t> visible_clusters;
        for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
            const Cluster& cluster = clusters[cluster_index];
            const bool visible = std::any_of(
                cluster.agent_indices.begin(), cluster.agent_indices.end(),
                [&](std::size_t member) {
                    return distance(
                               state.agents()[agent_index].position,
                               state.agents()[member].position) <=
                        state.agents()[agent_index].communication_radius;
                });
            if (visible) visible_clusters.push_back(cluster_index);
        }
        if (visible_clusters.size() < 2) continue;

        const auto leader_cluster = *std::min_element(
            visible_clusters.begin(), visible_clusters.end(),
            [&](std::size_t left, std::size_t right) {
                return clusters[left].leader_id < clusters[right].leader_id;
            });
        Cluster merged = clusters[leader_cluster];
        merged.agent_indices.push_back(agent_index);
        assigned[agent_index] = true;
        for (const std::size_t cluster_index : visible_clusters) {
            if (cluster_index == leader_cluster) continue;
            merged.agent_indices.insert(
                merged.agent_indices.end(),
                clusters[cluster_index].agent_indices.begin(),
                clusters[cluster_index].agent_indices.end());
        }
        std::sort(merged.agent_indices.begin(), merged.agent_indices.end());
        merged.agent_indices.erase(
            std::unique(merged.agent_indices.begin(), merged.agent_indices.end()),
            merged.agent_indices.end());

        std::vector<Cluster> next_clusters;
        for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
            if (std::find(visible_clusters.begin(), visible_clusters.end(), cluster_index) ==
                visible_clusters.end()) {
                next_clusters.push_back(clusters[cluster_index]);
            }
        }
        next_clusters.push_back(std::move(merged));
        clusters = std::move(next_clusters);
    }
    return clusters;
}

void aggregate_local_maps(const SimulationState& state, std::vector<Cluster>& clusters) {
    for (Cluster& cluster : clusters) {
        cluster.map_cells.clear();
        for (const std::size_t index : cluster.agent_indices) {
            const Agent& agent = state.agents()[index];
            for (const Position cell : state.environment().view(agent.position, agent.sensing_radius)) {
                cluster.map_cells[cell] = state.environment().traversable(cell);
            }
        }
        const ClusterLoad load = cluster_load(state, cluster.agent_indices);
        cluster.residual_capacity = load.residual_capacity;
        cluster.known_unresolved = load.known_unresolved;
        cluster.pressure = load.pressure;
    }
}

}  // namespace dmar
