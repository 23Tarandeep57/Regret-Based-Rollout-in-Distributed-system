#pragma once

#include "dmar/environment.hpp"
#include "dmar/state.hpp"

#include <cstddef>
#include <limits>
#include <vector>

namespace dmar {

struct Route {
    Position start;
    std::vector<Position> tasks;
};

double route_cost(const GridEnvironment& environment, const Route& route);
double cheapest_insertion_delta(const GridEnvironment& environment, const Route& route, Position task);

struct Assignment {
    std::size_t agent_index = 0;
    Position task;
    double unweighted_delta = 0.0;
    double weighted_cost = 0.0;
    double regret = 0.0;
    std::vector<Position> route_before;
    std::vector<Position> route_after;
};

struct RegretTask {
    Position task;
    double regret = 0.0;
    std::size_t eligible_agents = 0;
};

double task_regret(const std::vector<double>& deltas);
double cluster_pressure(std::size_t known_unresolved_tasks, int remaining_capacity);
std::vector<Assignment> capacity_weighted_assign(
    const GridEnvironment& environment,
    const std::vector<Agent>& agents,
    const std::vector<std::size_t>& participating_agents,
    const std::vector<Position>& tasks,
    double alpha = 0.5);

}  // namespace dmar