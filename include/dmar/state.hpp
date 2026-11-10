#pragma once

#include "dmar/environment.hpp"

#include <cstddef>
#include <vector>

namespace dmar {

struct Agent {
    int id = 0;
    Position position;
    int sensing_radius = 0;
    int communication_radius = 0;
    int initial_capacity = 0;
    int remaining_capacity = 0;

    bool can_perform_task() const noexcept { return remaining_capacity > 0; }
};

class SimulationState {
public:
    explicit SimulationState(GridEnvironment environment);

    GridEnvironment& environment() noexcept;
    const GridEnvironment& environment() const noexcept;
    std::vector<Agent>& agents() noexcept;
    const std::vector<Agent>& agents() const noexcept;
    void add_agent(Agent agent);
    bool complete_task(Agent& agent);
    void observe(const Agent& agent);
    bool observed(Position position) const noexcept;
    std::size_t observed_traversable_cells() const noexcept;
    std::size_t completed_tasks() const noexcept;
    std::size_t capacity_violations() const noexcept;

private:
    GridEnvironment environment_;
    std::vector<Agent> agents_;
    std::size_t completed_tasks_ = 0;
    std::size_t capacity_violations_ = 0;
    std::vector<bool> observed_;
};

}  // namespace dmar