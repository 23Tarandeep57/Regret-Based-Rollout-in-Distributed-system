#include "dmar/state.hpp"

#include <utility>

namespace dmar {

SimulationState::SimulationState(GridEnvironment environment)
    : environment_(std::move(environment)),
      observed_(static_cast<std::size_t>(environment_.rows() * environment_.columns()), false) {}

GridEnvironment& SimulationState::environment() noexcept { return environment_; }
const GridEnvironment& SimulationState::environment() const noexcept { return environment_; }
std::vector<Agent>& SimulationState::agents() noexcept { return agents_; }
const std::vector<Agent>& SimulationState::agents() const noexcept { return agents_; }

void SimulationState::add_agent(Agent agent) {
    if (agent.remaining_capacity < 0) agent.remaining_capacity = 0;
    if (agent.initial_capacity < agent.remaining_capacity) agent.initial_capacity = agent.remaining_capacity;
    agents_.push_back(agent);
}

bool SimulationState::complete_task(Agent& agent) {
    if (!environment_.has_task(agent.position)) return false;
    if (!agent.can_perform_task()) {
        ++capacity_violations_;
        return false;
    }
    environment_.remove_task(agent.position);
    --agent.remaining_capacity;
    ++completed_tasks_;
    return true;
}

std::size_t SimulationState::completed_tasks() const noexcept { return completed_tasks_; }
std::size_t SimulationState::capacity_violations() const noexcept { return capacity_violations_; }

void SimulationState::observe(const Agent& agent) {
    for (const Position cell : environment_.view(agent.position, agent.sensing_radius)) {
        const std::size_t cell_index =
            static_cast<std::size_t>(cell.row * environment_.columns() + cell.column);
        observed_[cell_index] = true;
    }
}

bool SimulationState::observed(Position position) const noexcept {
    if (!environment_.in_bounds(position)) return false;
    return observed_[static_cast<std::size_t>(position.row * environment_.columns() + position.column)];
}

std::size_t SimulationState::observed_traversable_cells() const noexcept {
    std::size_t count = 0;
    for (int row = 0; row < environment_.rows(); ++row) {
        for (int column = 0; column < environment_.columns(); ++column) {
            const Position cell{row, column};
            if (environment_.traversable(cell) && observed(cell)) ++count;
        }
    }
    return count;
}

}  // namespace dmar