#include "dmar/scenarios.hpp"
#include "dmar/simulator.hpp"

#include <iostream>

int main() {
    using namespace dmar;
    const MapDistribution maps[] = {
        MapDistribution::UniformObstacles, MapDistribution::Corridors};
    const TaskDistribution tasks[] = {
        TaskDistribution::Uniform, TaskDistribution::Clustered, TaskDistribution::Boundary};
    const AgentDistribution agents[] = {
        AgentDistribution::Uniform, AgentDistribution::Spread};

    std::cout << "map_distribution,task_distribution,agent_distribution,seed,"
                 "movement_cost,completed_tasks,unresolved_tasks,rounds\n";
    std::uint32_t seed = 1;
    for (const auto map : maps) {
        for (const auto task : tasks) {
            for (const auto agent : agents) {
                ScenarioConfig scenario;
                scenario.map_distribution = map;
                scenario.task_distribution = task;
                scenario.agent_distribution = agent;
                scenario.seed = seed;
                const Metrics metrics = run_simulation(generate_scenario(scenario), {});
                std::cout << to_string(map) << ',' << to_string(task) << ',' << to_string(agent) << ','
                          << seed << ',' << metrics.movement_cost << ',' << metrics.completed_tasks << ','
                          << metrics.unresolved_tasks << ',' << metrics.rounds << '\n';
                ++seed;
            }
        }
    }
}
