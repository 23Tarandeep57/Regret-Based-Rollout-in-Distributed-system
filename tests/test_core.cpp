#include "dmar/state.hpp"
#include "dmar/policies.hpp"
#include "dmar/rollout.hpp"
#include "dmar/simulator.hpp"
#include "dmar/scenarios.hpp"

#include <cassert>
#include <random>

using namespace dmar;

int main() {
    GridEnvironment environment(3, 3);
    environment.set_obstacle({1, 1});
    environment.add_task({0, 1});
    environment.add_task({2, 2});

    SimulationState state(std::move(environment));
    state.add_agent(Agent{1, {0, 0}, 1, 1, 1, 1});
    state.add_agent(Agent{2, {2, 1}, 1, 1, 0, 0});

    assert(state.environment().unresolved_tasks() == 2);
    const auto first_step = state.environment().shortest_step({0, 0}, {0, 1});
    assert(first_step.has_value());
    assert((*first_step).row == 0 && (*first_step).column == 1);
    assert(!state.environment().traversable({1, 1}));
    assert(state.agents()[0].remaining_capacity == 1);

    state.agents()[0].position = {0, 1};
    assert(state.complete_task(state.agents()[0]));
    assert(state.agents()[0].remaining_capacity == 0);
    assert(state.completed_tasks() == 1);
    assert(!state.complete_task(state.agents()[0]));

    state.agents()[1].position = {2, 2};
    assert(!state.complete_task(state.agents()[1]));
    assert(state.capacity_violations() == 1);

    std::mt19937 random(7);
    const Position next = state.environment().random_legal_move({0, 0}, random);
    assert((next.row == 0 && next.column == 1) || (next.row == 1 && next.column == 0));

    GridEnvironment route_environment(1, 5);
    const Route route{{0, 0}, {{0, 2}}};
    assert(route_cost(route_environment, route) == 2.0);
    assert(cheapest_insertion_delta(route_environment, route, {0, 4}) == 2.0);
    assert(task_regret({3.0, 7.0, 4.0}) == 1.0);
    assert(task_regret({3.0}) == 0.0);
    assert(cluster_pressure(3, 0) == 0.0);
    assert(cluster_pressure(3, 2) == 1.5);

    GridEnvironment rollout_environment(1, 4);
    SimulationState rollout_state(std::move(rollout_environment));
    rollout_state.add_agent(Agent{1, {0, 0}, 1, 1, 0, 0});
    rollout_state.add_agent(Agent{2, {0, 3}, 1, 1, 0, 0});
    const auto stay_policy = [](const SimulationState& current, std::size_t index) {
        return current.agents()[index].position;
    };
    const RolloutResult rollout = sequential_rollout(rollout_state, {0, 1}, stay_policy);
    assert(rollout.controls[0].row == 0 && rollout.controls[0].column == 0);
    assert(rollout.controls[1].row == 0 && rollout.controls[1].column == 3);
    const SimulationState copy = apply_controls(rollout_state, {{0, 1}, {0, 2}});
    assert(rollout_state.agents()[0].position.row == 0 && rollout_state.agents()[0].position.column == 0);
    assert(copy.agents()[0].position.row == 0 && copy.agents()[0].position.column == 1);

    GridEnvironment simulation_environment(1, 3);
    simulation_environment.add_task({0, 1});
    SimulationState simulation_state(std::move(simulation_environment));
    simulation_state.add_agent(Agent{1, {0, 0}, 1, 1, 1, 1});
    SimulatorConfig simulation_config;
    simulation_config.max_rounds = 4;
    simulation_config.seed = 11;
    const Metrics first_run = run_simulation(simulation_state, simulation_config);
    const Metrics second_run = run_simulation(simulation_state, simulation_config);
    assert(first_run.movement_cost == second_run.movement_cost);
    assert(first_run.completed_tasks == second_run.completed_tasks);
    assert(first_run.capacity_violations == 0);

    GridEnvironment capacity_environment(1, 3);
    capacity_environment.add_task({0, 1});
    capacity_environment.add_task({0, 2});
    SimulationState capacity_state(std::move(capacity_environment));
    capacity_state.add_agent(Agent{1, {0, 0}, 1, 1, 1, 1});
    SimulatorConfig capacity_config;
    capacity_config.algorithm = Algorithm::GreedyExploration;
    capacity_config.max_rounds = 4;
    const Metrics capacity_run = run_simulation(capacity_state, capacity_config);
    assert(capacity_run.completed_tasks == 1);
    assert(capacity_run.unresolved_tasks == 1);
    assert(capacity_run.capacity_violations == 0);

    GridEnvironment zero_capacity_environment(1, 2);
    zero_capacity_environment.add_task({0, 1});
    SimulationState zero_capacity_state(std::move(zero_capacity_environment));
    zero_capacity_state.add_agent(Agent{1, {0, 0}, 1, 1, 0, 0});
    SimulatorConfig zero_capacity_config;
    zero_capacity_config.algorithm = Algorithm::GreedyExploration;
    zero_capacity_config.max_rounds = 2;
    const Metrics zero_capacity_run = run_simulation(zero_capacity_state, zero_capacity_config);
    assert(zero_capacity_run.completed_tasks == 0);
    assert(zero_capacity_run.unresolved_tasks == 1);
    assert(zero_capacity_run.capacity_violations == 0);

    ScenarioConfig scenario;
    scenario.rows = 10;
    scenario.columns = 10;
    scenario.task_count = 8;
    scenario.agent_count = 3;
    scenario.seed = 42;
    scenario.map_distribution = MapDistribution::Corridors;
    scenario.task_distribution = TaskDistribution::Boundary;
    scenario.agent_distribution = AgentDistribution::Spread;
    const SimulationState generated = generate_scenario(scenario);
    assert(generated.agents().size() == 3);
    assert(generated.environment().unresolved_tasks() == 8);
    for (const Agent& agent : generated.agents()) {
        assert(generated.environment().traversable(agent.position));
    }
    const SimulationState repeated = generate_scenario(scenario);
    assert(repeated.agents()[0].position == generated.agents()[0].position);
    assert(repeated.environment().unresolved_tasks() == generated.environment().unresolved_tasks());

    GridEnvironment frontier_environment(3, 5);
    frontier_environment.set_obstacle({0, 2});
    frontier_environment.set_obstacle({1, 2});
    frontier_environment.set_obstacle({2, 2});
    SimulationState frontier_state(std::move(frontier_environment));
    frontier_state.add_agent(Agent{1, {1, 1}, 0, 1, 0, 0});
    frontier_state.observe(frontier_state.agents()[0]);
    const Position frontier_step = deterministic_frontier_step(frontier_state, 0);
    assert((frontier_step == Position{0, 1}));
    frontier_state.agents()[0].position = {1, 3};
    assert(deterministic_frontier_step(frontier_state, 0) == frontier_state.agents()[0].position);

    ScenarioConfig capacity_scenario;
    capacity_scenario.rows = 6;
    capacity_scenario.columns = 6;
    capacity_scenario.task_count = 3;
    capacity_scenario.agent_count = 3;
    capacity_scenario.agent_capacities = {0, 1, 2};
    const SimulationState capacity_generated = generate_scenario(capacity_scenario);
    assert(capacity_generated.agents()[0].initial_capacity == 0);
    assert(capacity_generated.agents()[1].initial_capacity == 1);
    assert(capacity_generated.agents()[2].initial_capacity == 2);
    return 0;
}