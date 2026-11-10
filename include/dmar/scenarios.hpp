#pragma once

#include "dmar/state.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dmar {

enum class MapDistribution { UniformObstacles, Corridors };
enum class TaskDistribution { Uniform, Clustered, Boundary };
enum class AgentDistribution { Uniform, Spread };

struct ScenarioConfig {
    int rows = 20;
    int columns = 20;
    double obstacle_probability = 0.15;
    std::size_t task_count = 30;
    std::size_t agent_count = 5;
    int sensing_radius = 2;
    int communication_radius = 2;
    int agent_capacity = 100;
    std::vector<int> agent_capacities;
    MapDistribution map_distribution = MapDistribution::UniformObstacles;
    TaskDistribution task_distribution = TaskDistribution::Uniform;
    AgentDistribution agent_distribution = AgentDistribution::Uniform;
    std::uint32_t seed = 1;
};

SimulationState generate_scenario(const ScenarioConfig& config);
std::string to_string(MapDistribution distribution);
std::string to_string(TaskDistribution distribution);
std::string to_string(AgentDistribution distribution);

}  // namespace dmar
