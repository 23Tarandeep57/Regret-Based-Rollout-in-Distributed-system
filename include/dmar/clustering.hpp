#pragma once

#include "dmar/state.hpp"

#include <map>
#include <vector>

namespace dmar {

struct PositionLess {
    bool operator()(Position left, Position right) const {
        return left.row != right.row ? left.row < right.row : left.column < right.column;
    }
};

struct Cluster {
    int leader_id = 0;
    std::vector<std::size_t> agent_indices;
    std::map<Position, bool, PositionLess> map_cells;
    // Filled by aggregate_local_maps (capacity-aware planning state, spec section 3).
    int residual_capacity = 0;          // R_K = sum of r_i over members
    std::size_t known_unresolved = 0;   // N_K = unresolved tasks in the union of member views
    double pressure = 0.0;              // P_K = N_K / R_K (0 when R_K == 0)
};

// `capacity_aware` enables both CA-SOAC modifications: only agents with r_i > 0 may become
// leaders, and an agent choosing among several joinable clusters prefers the highest
// pressure P_K = N_K / R_K (ties broken by lowest leader ID). When false, behaves as the
// capacity-unaware baseline.
std::vector<Cluster> form_soac_clusters(
    const SimulationState& state, int psi, int child_limit, bool capacity_aware = false);
void aggregate_local_maps(const SimulationState& state, std::vector<Cluster>& clusters);

}  // namespace dmar