#pragma once

#include "dmar/state.hpp"

#include <functional>
#include <random>
#include <vector>

namespace dmar {

using Control = Position;
using BasePolicy = std::function<Control(const SimulationState&, std::size_t)>;

struct RolloutConfig {
    int horizon = 30;
    std::vector<Position> known_tasks;
};

struct RolloutResult {
    std::vector<Control> controls;
    double estimated_cost = 0.0;
};

SimulationState apply_controls(const SimulationState& state, const std::vector<Control>& controls);
RolloutResult sequential_rollout(
    const SimulationState& state,
    const std::vector<std::size_t>& agent_order,
    const BasePolicy& base_policy,
    RolloutConfig config = {});

}  // namespace dmar