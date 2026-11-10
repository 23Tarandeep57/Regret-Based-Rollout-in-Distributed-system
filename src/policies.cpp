#include "dmar/policies.hpp"

#include <algorithm>
#include <cmath>

namespace dmar {

namespace {

double leg_cost(const GridEnvironment& environment, Position from, Position to) {
    const auto distance = environment.shortest_distance(from, to);
    return distance.has_value() ? static_cast<double>(*distance) : std::numeric_limits<double>::infinity();
}

bool position_less(Position left, Position right) {
    return left.row != right.row ? left.row < right.row : left.column < right.column;
}

}  // namespace

double route_cost(const GridEnvironment& environment, const Route& route) {
    double total = 0.0;
    Position current = route.start;
    for (const Position task : route.tasks) {
        total += leg_cost(environment, current, task);
        current = task;
    }
    return total;
}

double cheapest_insertion_delta(const GridEnvironment& environment, const Route& route, Position task) {
    const double original = route_cost(environment, route);
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t insertion = 0; insertion <= route.tasks.size(); ++insertion) {
        Route candidate = route;
        candidate.tasks.insert(candidate.tasks.begin() + static_cast<std::ptrdiff_t>(insertion), task);
        best = std::min(best, route_cost(environment, candidate));
    }
    return best - original;
}

double task_regret(const std::vector<double>& deltas) {
    if (deltas.size() < 2) return 0.0;
    std::vector<double> sorted = deltas;
    std::sort(sorted.begin(), sorted.end());
    return sorted[1] - sorted[0];
}

double cluster_pressure(std::size_t known_unresolved_tasks, int remaining_capacity) {
    if (remaining_capacity <= 0) return 0.0;
    return static_cast<double>(known_unresolved_tasks) / static_cast<double>(remaining_capacity);
}

std::vector<Assignment> capacity_weighted_assign(
    const GridEnvironment& environment,
    const std::vector<Agent>& agents,
    const std::vector<std::size_t>& participating_agents,
    const std::vector<Position>& tasks,
    double alpha) {
    if (alpha < 0.0 || alpha > 1.0) return {};
    int total_capacity = 0;
    std::vector<int> provisional_capacity(agents.size(), 0);
    for (const std::size_t index : participating_agents) {
        provisional_capacity[index] = agents[index].can_perform_task()
            ? agents[index].remaining_capacity
            : 0;
        total_capacity += provisional_capacity[index];
    }
    if (total_capacity == 0) return {};

    std::vector<Route> routes;
    routes.reserve(agents.size());
    for (const Agent& agent : agents) routes.push_back(Route{agent.position, {}});

    std::vector<Position> pending = tasks;
    std::vector<Assignment> result;
    while (!pending.empty()) {
        std::vector<RegretTask> ordering;
        for (const Position task : pending) {
            std::vector<double> deltas;
            for (const std::size_t index : participating_agents) {
                if (provisional_capacity[index] <= 0) continue;
                const double delta = cheapest_insertion_delta(environment, routes[index], task);
                if (std::isfinite(delta)) deltas.push_back(delta);
            }
            if (!deltas.empty()) ordering.push_back(RegretTask{task, task_regret(deltas), deltas.size()});
        }
        if (ordering.empty()) break;
        // Spec 4: tasks with a single eligible agent go first (no alternative exists), then
        // tasks with larger regret, then a deterministic positional tie-break.
        std::sort(ordering.begin(), ordering.end(), [](const RegretTask& left, const RegretTask& right) {
            const bool left_single = left.eligible_agents == 1;
            const bool right_single = right.eligible_agents == 1;
            if (left_single != right_single) return left_single;
            if (left.regret != right.regret) return left.regret > right.regret;
            return position_less(left.task, right.task);
        });
        const Position task = ordering.front().task;
        const double selected_regret = ordering.front().regret;

        std::optional<Assignment> best;
        for (const std::size_t index : participating_agents) {
            if (provisional_capacity[index] <= 0) continue;
            const double delta = cheapest_insertion_delta(environment, routes[index], task);
            if (!std::isfinite(delta)) continue;
            const double factor = std::pow(static_cast<double>(total_capacity) /
                                               static_cast<double>(provisional_capacity[index]), alpha);
            const Assignment candidate{
                index, task, delta, delta * factor, selected_regret, routes[index].tasks, {}};
            if (!best.has_value() || candidate.weighted_cost < best->weighted_cost ||
                (candidate.weighted_cost == best->weighted_cost && index < best->agent_index)) {
                best = candidate;
            }
        }
        if (!best.has_value()) break;
        std::size_t insertion = 0;
        double insertion_cost = std::numeric_limits<double>::infinity();
        for (std::size_t candidate = 0; candidate <= routes[best->agent_index].tasks.size(); ++candidate) {
            Route route = routes[best->agent_index];
            route.tasks.insert(route.tasks.begin() + static_cast<std::ptrdiff_t>(candidate), task);
            const double cost = route_cost(environment, route);
            if (cost < insertion_cost) {
                insertion = candidate;
                insertion_cost = cost;
            }
        }
        routes[best->agent_index].tasks.insert(
            routes[best->agent_index].tasks.begin() + static_cast<std::ptrdiff_t>(insertion), task);
        best->route_after = routes[best->agent_index].tasks;
        --provisional_capacity[best->agent_index];
        --total_capacity;
        result.push_back(*best);
        pending.erase(std::find(pending.begin(), pending.end(), task));
    }
    return result;
}

}  // namespace dmar