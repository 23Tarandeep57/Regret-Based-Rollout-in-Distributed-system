#include "dmar/scenarios.hpp"
#include "dmar/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool solvable(const dmar::SimulationState& state) {
    for (int row = 0; row < state.environment().rows(); ++row) {
        for (int column = 0; column < state.environment().columns(); ++column) {
            const dmar::Position task{row, column};
            if (!state.environment().has_task(task)) continue;
            bool reachable = false;
            for (const dmar::Agent& agent : state.agents()) {
                if (state.environment().shortest_distance(agent.position, task).has_value()) {
                    reachable = true;
                    break;
                }
            }
            if (!reachable) return false;
        }
    }
    return true;
}

dmar::AssignmentPolicy assignment_policy(int value) {
    return value == 1
        ? dmar::AssignmentPolicy::GreedyCapacity
        : dmar::AssignmentPolicy::CapacityWeighted;
}

const char* algorithm_name(dmar::Algorithm algorithm) {
    switch (algorithm) {
    case dmar::Algorithm::GreedyExploration: return "greedy";
    case dmar::Algorithm::CA_DMAR_RW: return "ca_dmar_r";
    case dmar::Algorithm::CC_DMAR: return "dmar";
    case dmar::Algorithm::CC_DMAR_LE: return "dmar_le";
    }
    return "unknown";
}

int argument_or_default(int argc, char** argv, int index, int fallback) {
    return argc > index ? std::atoi(argv[index]) : fallback;
}

double double_argument_or_default(int argc, char** argv, int index, double fallback) {
    return argc > index ? std::atof(argv[index]) : fallback;
}

int movement_total(const std::vector<int>& values) {
    int total = 0;
    for (const int value : values) total += value;
    return total;
}

dmar::ExhaustedExploration exploration_mode(int value) {
    switch (value) {
    case 1: return dmar::ExhaustedExploration::Stop;
    case 2: return dmar::ExhaustedExploration::Frontier;
    default: return dmar::ExhaustedExploration::Random;
    }
}

dmar::SoacPolicy soac_policy(int value) {
    switch (value) {
    case 1: return dmar::SoacPolicy::CommunicationOnly;
    case 2: return dmar::SoacPolicy::OriginalStyleEligible;
    case 3: return dmar::SoacPolicy::OriginalStyleAllMembers;
    default: return dmar::SoacPolicy::CurrentCapacityAware;
    }
}

}  // namespace

int main(int argc, char** argv) {
    using namespace dmar;
    const int max_side = argument_or_default(argc, argv, 1, 80);
    const int instances = argument_or_default(argc, argv, 2, 10);
    const int runs = argument_or_default(argc, argv, 3, 10);
    const int max_rounds = argument_or_default(argc, argv, 4, 10000);
    const int max_sensing_radius = argument_or_default(argc, argv, 5, 12);
    const double alpha = double_argument_or_default(argc, argv, 6, 0.5);
    const int exhausted_exploration = argument_or_default(argc, argv, 7, 1);
    const bool debug_logging = argument_or_default(argc, argv, 8, 0) != 0;
    const int soac_mode = argument_or_default(argc, argv, 9, 0);
    const int assignment_mode = argument_or_default(argc, argv, 10, 0);
    const int trace_side = argument_or_default(argc, argv, 11, 0);
    const int trace_ratio = argument_or_default(argc, argv, 12, -1);
    const int trace_radius = argument_or_default(argc, argv, 13, 0);
    const int algorithm_mode = argument_or_default(argc, argv, 14, 0);
    const double custom_tasks_per_agent = double_argument_or_default(argc, argv, 15, -1.0);
    const int min_side = argument_or_default(argc, argv, 16, 10);
    const int custom_agent_count = argument_or_default(argc, argv, 17, -1);
    const int capacity_mode = argument_or_default(argc, argv, 18, 0);
    if (max_side < 10 || min_side < 10 || min_side > max_side ||
        min_side % 10 != 0 || max_side % 10 != 0 ||
        instances <= 0 || runs <= 0 || max_rounds <= 0 ||
        max_sensing_radius <= 0 || alpha < 0.0 || alpha > 1.0 ||
        algorithm_mode < 0 || algorithm_mode > 4 ||
        (custom_tasks_per_agent != -1.0 && custom_tasks_per_agent <= 0.0) ||
        (custom_agent_count != -1 && custom_agent_count <= 0) ||
        (capacity_mode < 0 || capacity_mode > 3)) {
        std::cerr << "usage: dmar_benchmark [max_side>=10] [instances] [runs] [max_rounds] "
                     "[max_sensing_radius] [alpha in 0..1] "
                     "[exhausted_mode 0=random,1=stop,2=frontier] [debug] "
                     "[soac_mode 0=current,1=communication-only,2=all-members-eligible,3=all-members-planning] "
                     "[assignment_mode 0=capacity-weighted,1=greedy-capacity] "
                     "[trace_side] [trace_ratio_index] [trace_radius] "
                     "[algorithm_mode 0=all,1=CA-DMAR-R-only,2=non-CA-only,3=DMAR-LE-only,4=DMAR-LE+CA-DMAR-R] "
                     "[tasks_per_agent optional custom ratio] "
                     "[min_side optional focused start size] "
                     "[agent_count optional custom count] "
                     "[capacity_mode 0=uniform,1=heterogeneous,2=moderate-spread,3=high-spread]\n";
        return EXIT_FAILURE;
    }

    const int total_sizes = (max_side - min_side) / 10 + 1;
    const int ratio_count = custom_tasks_per_agent > 0.0 ? 1 : 3;
    const int total_configs = total_sizes * ratio_count * instances * max_sensing_radius;
    std::cerr << "DMAR benchmark\n"
              << "  grid sizes: 10.." << max_side << "\n"
              << "  instances/ratio: " << instances << "\n"
              << "  runs/instance: " << runs << "\n"
              << "  sensing radii: 1.." << max_sensing_radius << "\n"
              << "  tasks per agent: "
              << (custom_tasks_per_agent > 0.0 ? custom_tasks_per_agent : 0.0) << "\n"
              << "  CA-DMAR-R alpha: " << alpha << "\n"
              << "  capacity mode: " << capacity_mode << "\n"
              << "  exhausted exploration mode: " << exhausted_exploration << "\n"
              << "  configurations: " << total_configs << "\n";
    std::cout << "algorithm,side,instance,run,sensing_radius,agent_count,task_count,"
                 "task_agent_ratio,seed,movement_cost,completed_tasks,unresolved_tasks,rounds,"
                 "initial_capacity,remaining_capacity,capacity_violations,alpha,"
                 "execution_capacity_violations,rejected_assignments,simulated_capacity_rejections,"
                 "eligible_movement,exhausted_movement,task_serving_movement,exploration_movement,"
                 "exhausted_clustered_movement,exhausted_unclustered_movement,"
                 "observed_traversable_cells,total_traversable_cells,exhausted_exploration,"
                 "soac_mode,assignment_mode,cluster_memberships,exhausted_cluster_memberships,"
                 "max_cluster_size,unique_clustered_agents,rounds_with_clusters,"
                 "cluster_dissolutions,exhausted_clustered_rounds,exhausted_agent_rounds,"
                 "tasks_seen_by_exhausted_agents\n";
    const double task_agent_ratios[] = {2.0, 1.0, 0.5};
    const char* ratio_labels[] = {"1:2", "1:1", "2:1"};
    for (int side = min_side; side <= max_side; side += 10) {
        for (int ratio_index = 0; ratio_index < ratio_count; ++ratio_index) {
            const double ratio = custom_tasks_per_agent > 0.0
                ? custom_tasks_per_agent
                : task_agent_ratios[ratio_index];
            const std::string custom_ratio_label =
                "1:" + std::to_string(static_cast<int>(custom_tasks_per_agent));
            const char* ratio_label = custom_tasks_per_agent > 0.0
                ? custom_ratio_label.c_str()
                : ratio_labels[ratio_index];
            const int agent_count = custom_agent_count > 0 ? custom_agent_count : side;
            const auto task_count = static_cast<std::size_t>(std::lround(agent_count * ratio));
            for (int instance = 0; instance < instances; ++instance) {
                ScenarioConfig scenario;
                scenario.rows = side;
                scenario.columns = side;
                scenario.obstacle_probability = 0.20;
                scenario.agent_count = static_cast<std::size_t>(agent_count);
                scenario.task_count = task_count;
                scenario.map_distribution = MapDistribution::UniformObstacles;
                scenario.task_distribution = TaskDistribution::Uniform;
                scenario.agent_distribution = AgentDistribution::Uniform;
                scenario.agent_capacities.resize(scenario.agent_count);
                for (std::size_t index = 0; index < scenario.agent_count; ++index) {
                    const int uniform_capacity =
                        static_cast<int>(task_count / scenario.agent_count) +
                        (index < task_count % scenario.agent_count ? 1 : 0);
                    if (capacity_mode == 0) {
                        scenario.agent_capacities[index] = uniform_capacity;
                    } else if (capacity_mode == 2 && scenario.agent_count == 10) {
                        const int capacities[] = {4, 4, 5, 5, 5, 5, 6, 6, 6, 8};
                        scenario.agent_capacities[index] = capacities[index];
                    } else if (capacity_mode == 3 && scenario.agent_count == 10) {
                        const int capacities[] = {1, 2, 3, 4, 5, 6, 8, 10, 14, 20};
                        scenario.agent_capacities[index] = capacities[index];
                    } else {
                        const int centered_offset =
                            static_cast<int>(index) -
                            static_cast<int>(scenario.agent_count / 2);
                        scenario.agent_capacities[index] =
                            std::max(1, uniform_capacity + centered_offset);
                    }
                }
                if (capacity_mode == 1) {
                    const int slack = std::max(1, static_cast<int>(task_count / 5));
                    scenario.agent_capacities.back() += slack;
                }
                scenario.seed = static_cast<std::uint32_t>(
                    side * 100000 + instance * 1000 + ratio_index * 100);
                SimulationState state = generate_scenario(scenario);
                int attempts = 0;
                while (!solvable(state) && attempts++ < 1000) {
                    ++scenario.seed;
                    state = generate_scenario(scenario);
                }
                if (!solvable(state)) {
                    std::cerr << "unable to generate a solvable instance after 1000 attempts\n";
                    return EXIT_FAILURE;
                }
                std::cerr << "instance side=" << side << " ratio=" << ratio_label
                          << " index=" << (instance + 1) << '/' << instances
                          << " seed=" << scenario.seed << '\n';
                for (int sensing_radius = 1; sensing_radius <= max_sensing_radius; ++sensing_radius) {
                    scenario.sensing_radius = sensing_radius;
                    scenario.communication_radius = sensing_radius;
                    state = generate_scenario(scenario);
                    for (int run = 0; run < runs; ++run) {
                        for (const auto algorithm : {Algorithm::GreedyExploration, Algorithm::CC_DMAR,
                                                     Algorithm::CC_DMAR_LE, Algorithm::CA_DMAR_RW}) {
                            const bool is_ca = algorithm == Algorithm::CA_DMAR_RW;
                            const bool is_dmar_le = algorithm == Algorithm::CC_DMAR_LE;
                            if ((algorithm_mode == 1 && !is_ca) ||
                                (algorithm_mode == 2 && is_ca) ||
                                (algorithm_mode == 3 && !is_dmar_le) ||
                                (algorithm_mode == 4 && !is_ca && !is_dmar_le)) {
                                continue;
                            }
                            SimulatorConfig config;
                            config.algorithm = algorithm;
                            config.alpha = alpha;
                            config.psi = 8;
                            config.child_limit = 4;
                            config.max_rounds = max_rounds;
                            config.seed = scenario.seed + static_cast<std::uint32_t>(run);
                            config.exhausted_exploration =
                                algorithm == Algorithm::CA_DMAR_RW
                                ? exploration_mode(exhausted_exploration)
                                : ExhaustedExploration::Random;
                            config.debug_logging = debug_logging &&
                                (trace_side == 0 || trace_side == side) &&
                                (trace_ratio < 0 || trace_ratio == static_cast<int>(ratio_index)) &&
                                (trace_radius == 0 || trace_radius == sensing_radius);
                            config.soac_policy = soac_policy(soac_mode);
                            config.assignment_policy = assignment_policy(assignment_mode);
                            if (debug_logging) std::cerr << "algorithm=" << algorithm_name(algorithm)
                                                         << " radius=" << sensing_radius
                                                         << " run=" << run << '\n';
                            const Metrics metrics = run_simulation(state, config);
                            std::cout << algorithm_name(algorithm) << ',' << side << ',' << instance << ','
                                      << run << ',' << sensing_radius << ',' << agent_count << ','
                                      << task_count << ',' << ratio_label << ','
                                      << config.seed << ',' << metrics.movement_cost << ','
                                      << metrics.completed_tasks << ',' << metrics.unresolved_tasks << ','
                                      << metrics.rounds << ',' << metrics.initial_capacity << ','
                                      << metrics.remaining_capacity << ',' << metrics.capacity_violations
                                      << ',' << alpha << ','
                                      << metrics.execution_capacity_violations << ','
                                      << metrics.rejected_assignments << ','
                                      << metrics.simulated_capacity_rejections << ','
                                      << movement_total(metrics.eligible_movement_by_agent) << ','
                                      << movement_total(metrics.exhausted_movement_by_agent) << ','
                                      << movement_total(metrics.task_serving_movement_by_agent) << ','
                                      << movement_total(metrics.exploration_movement_by_agent) << ','
                                      << movement_total(metrics.exhausted_clustered_movement_by_agent) << ','
                                      << movement_total(metrics.exhausted_unclustered_movement_by_agent) << ','
                                      << metrics.observed_traversable_cells << ','
                                      << metrics.total_traversable_cells << ','
                                      << exhausted_exploration << ','
                                      << soac_mode << ','
                                      << assignment_mode << ','
                                      << metrics.cluster_memberships << ','
                                      << metrics.exhausted_cluster_memberships << ','
                                      << metrics.max_cluster_size << ','
                                      << metrics.unique_clustered_agents << ','
                                      << metrics.rounds_with_clusters << ','
                                      << metrics.cluster_dissolutions << ','
                                      << metrics.exhausted_clustered_rounds << ','
                                      << metrics.exhausted_agent_rounds << ','
                                      << metrics.tasks_seen_by_exhausted_agents << '\n';
                        }
                    }
                }
                std::cerr << "completed side=" << side << " ratio=" << ratio_label << '\n';
            }
        }
        std::cerr << "benchmark complete; CSV results written to stdout\n";
    }
}
