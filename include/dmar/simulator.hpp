#pragma once

#include "dmar/clustering.hpp"
#include "dmar/policies.hpp"
#include "dmar/rollout.hpp"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <vector>

namespace dmar {

enum class Algorithm { CC_DMAR, GreedyExploration, CA_DMAR_RW };
enum class ExhaustedExploration { Random, Stop, Frontier };
enum class SoacPolicy {
    CurrentCapacityAware,
    CommunicationOnly,
    OriginalStyleEligible,
    OriginalStyleAllMembers
};
enum class AssignmentPolicy { CapacityWeighted, GreedyCapacity };

struct SimulatorConfig {
    Algorithm algorithm = Algorithm::CC_DMAR;
    int psi = 4;
    int child_limit = 4;
    int max_rounds = 1000;
    double alpha = 0.5;
    std::uint32_t seed = 1;
    bool debug_logging = false;
    ExhaustedExploration exhausted_exploration = ExhaustedExploration::Random;
    SoacPolicy soac_policy = SoacPolicy::CurrentCapacityAware;
    AssignmentPolicy assignment_policy = AssignmentPolicy::CapacityWeighted;
};

struct Metrics {
    double movement_cost = 0.0;
    std::size_t completed_tasks = 0;
    std::size_t unresolved_tasks = 0;
    std::size_t capacity_violations = 0;
    std::size_t execution_capacity_violations = 0;
    std::size_t rejected_assignments = 0;
    std::size_t simulated_capacity_rejections = 0;
    int initial_capacity = 0;
    int remaining_capacity = 0;
    int rounds = 0;
    std::vector<std::size_t> completed_tasks_by_agent;
    std::vector<int> movement_by_agent;
    std::vector<int> eligible_movement_by_agent;
    std::vector<int> exhausted_movement_by_agent;
    std::vector<int> exhausted_clustered_movement_by_agent;
    std::vector<int> exhausted_unclustered_movement_by_agent;
    std::vector<int> task_serving_movement_by_agent;
    std::vector<int> exploration_movement_by_agent;
    std::size_t observed_traversable_cells = 0;
    std::size_t total_traversable_cells = 0;
    std::size_t cluster_memberships = 0;
    std::size_t exhausted_cluster_memberships = 0;
    std::size_t max_cluster_size = 0;
    std::size_t rounds_with_clusters = 0;
    std::size_t cluster_dissolutions = 0;
    std::size_t exhausted_clustered_rounds = 0;
    std::size_t exhausted_agent_rounds = 0;
    std::size_t tasks_seen_by_exhausted_agents = 0;
    std::size_t unique_clustered_agents = 0;
};

Control deterministic_frontier_step(const SimulationState& state, std::size_t agent_index);
Metrics run_simulation(SimulationState state, const SimulatorConfig& config);

}  // namespace dmar