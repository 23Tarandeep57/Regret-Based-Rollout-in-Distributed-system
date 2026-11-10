#include "dmar/scenarios.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace dmar {

namespace {

bool boundary(Position position, int rows, int columns) {
    return position.row == 0 || position.column == 0 || position.row == rows - 1 ||
           position.column == columns - 1;
}

std::vector<Position> traversable_cells(const GridEnvironment& environment) {
    std::vector<Position> cells;
    for (int row = 0; row < environment.rows(); ++row) {
        for (int column = 0; column < environment.columns(); ++column) {
            Position position{row, column};
            if (environment.traversable(position)) cells.push_back(position);
        }
    }
    return cells;
}

template <typename Predicate>
std::vector<Position> filtered(const std::vector<Position>& cells, Predicate predicate) {
    std::vector<Position> result;
    for (const Position position : cells) {
        if (predicate(position)) result.push_back(position);
    }
    return result;
}

std::vector<Position> choose_cells(
    std::vector<Position> candidates, std::size_t count, std::mt19937& random) {
    if (count > candidates.size()) {
        throw std::invalid_argument("scenario requests more cells than are available");
    }
    std::shuffle(candidates.begin(), candidates.end(), random);
    candidates.resize(count);
    return candidates;
}

void add_obstacles(const ScenarioConfig& config, GridEnvironment& environment, std::mt19937& random) {
    if (config.map_distribution == MapDistribution::Corridors) {
        for (int row = 1; row < config.rows - 1; ++row) {
            if (row % 4 == 0) {
                for (int column = 1; column < config.columns - 1; ++column) {
                    if (column % 7 != 1) environment.set_obstacle({row, column});
                }
            }
        }
        return;
    }

    std::bernoulli_distribution blocked(config.obstacle_probability);
    for (int row = 0; row < config.rows; ++row) {
        for (int column = 0; column < config.columns; ++column) {
            if (blocked(random)) {
                environment.set_obstacle({row, column});
            }
        }
    }
}

std::vector<Position> task_candidates(
    const ScenarioConfig& config, std::vector<Position> cells, std::mt19937& random) {
    if (config.task_distribution == TaskDistribution::Boundary) {
        auto boundary_cells = filtered(cells, [&](Position position) {
            return boundary(position, config.rows, config.columns);
        });
        if (boundary_cells.size() >= config.task_count) return boundary_cells;
    }
    if (config.task_distribution == TaskDistribution::Clustered) {
        const Position center{
            std::uniform_int_distribution<int>(1, config.rows - 2)(random),
            std::uniform_int_distribution<int>(1, config.columns - 2)(random)};
        std::sort(cells.begin(), cells.end(), [&](Position left, Position right) {
            const auto distance = [&](Position position) {
                return std::abs(position.row - center.row) + std::abs(position.column - center.column);
            };
            return distance(left) < distance(right);
        });
        const std::size_t sample_pool = std::min(
            cells.size(), std::max(config.task_count, config.task_count * 3));
        cells.resize(sample_pool);
    }
    return cells;
}

std::vector<Position> agent_candidates(
    const ScenarioConfig& config, const std::vector<Position>& cells, std::mt19937& random) {
    if (config.agent_distribution == AgentDistribution::Spread) {
        std::vector<Position> result;
        result.reserve(config.agent_count);
        for (std::size_t index = 0; index < config.agent_count; ++index) {
            const double fraction = config.agent_count == 1
                ? 0.0
                : static_cast<double>(index) / static_cast<double>(config.agent_count - 1);
            const std::size_t candidate =
                static_cast<std::size_t>(fraction * static_cast<double>(cells.size() - 1));
            result.push_back(cells[candidate]);
        }
        return result;
    }
    return choose_cells(cells, config.agent_count, random);
}

}  // namespace

SimulationState generate_scenario(const ScenarioConfig& config) {
    if (config.rows < 3 || config.columns < 3 || config.obstacle_probability < 0.0 ||
        config.obstacle_probability > 1.0 || config.agent_capacity < 0 ||
        (!config.agent_capacities.empty() &&
         config.agent_capacities.size() != config.agent_count)) {
        throw std::invalid_argument("invalid scenario dimensions or probabilities");
    }
    for (const int capacity : config.agent_capacities) {
        if (capacity < 0) throw std::invalid_argument("agent capacity cannot be negative");
    }
    std::mt19937 random(config.seed);
    GridEnvironment environment(config.rows, config.columns);
    add_obstacles(config, environment, random);
    auto cells = traversable_cells(environment);
    if (cells.size() < config.task_count + config.agent_count) {
        throw std::invalid_argument("scenario does not have enough traversable cells");
    }

    const auto tasks = choose_cells(task_candidates(config, cells, random), config.task_count, random);
    for (const Position task : tasks) environment.add_task(task);

    SimulationState state(std::move(environment));
    const auto agents = agent_candidates(config, cells, random);
    for (std::size_t index = 0; index < agents.size(); ++index) {
        const int capacity = config.agent_capacities.empty()
            ? config.agent_capacity
            : config.agent_capacities[index];
        state.add_agent(Agent{
            static_cast<int>(index + 1), agents[index], config.sensing_radius,
            config.communication_radius, capacity, capacity});
    }
    return state;
}

std::string to_string(MapDistribution distribution) {
    return distribution == MapDistribution::UniformObstacles ? "uniform" : "corridors";
}

std::string to_string(TaskDistribution distribution) {
    switch (distribution) {
    case TaskDistribution::Uniform: return "uniform";
    case TaskDistribution::Clustered: return "clustered";
    case TaskDistribution::Boundary: return "boundary";
    }
    return "unknown";
}

std::string to_string(AgentDistribution distribution) {
    return distribution == AgentDistribution::Uniform ? "uniform" : "spread";
}

}  // namespace dmar
