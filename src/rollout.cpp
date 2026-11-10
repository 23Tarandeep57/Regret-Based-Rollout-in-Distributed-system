#include "dmar/rollout.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace dmar {

namespace {

Control base_control(const SimulationState& state, std::size_t agent_index, const BasePolicy& policy) {
    const Agent& agent = state.agents()[agent_index];
    const Control selected = policy(state, agent_index);
    for (const Position legal : state.environment().legal_moves(agent.position)) {
        if (legal == selected) return selected;
    }
    return agent.position;
}

double movement_cost(const SimulationState& before, const SimulationState& after) {
    double cost = 0.0;
    for (std::size_t index = 0; index < before.agents().size(); ++index) {
        cost += before.agents()[index].position == after.agents()[index].position ? 0.0 : 1.0;
    }
    return cost;
}

int remaining_known(
    const SimulationState& state, const std::vector<Position>& known_tasks) {
    int remaining = 0;
    for (const Position task : known_tasks) {
        if (state.environment().has_task(task)) ++remaining;
    }
    return remaining;
}

}  // namespace

SimulationState apply_controls(const SimulationState& state, const std::vector<Control>& controls) {
    SimulationState next = state;
    for (std::size_t index = 0; index < next.agents().size(); ++index) {
        const Position requested = index < controls.size() ? controls[index] : next.agents()[index].position;
        const auto legal_moves = next.environment().legal_moves(next.agents()[index].position);
        if (std::find(legal_moves.begin(), legal_moves.end(), requested) != legal_moves.end()) {
            next.agents()[index].position = requested;
        }
    }
    for (Agent& agent : next.agents()) {
        if (agent.can_perform_task()) next.complete_task(agent);
    }
    return next;
}

RolloutResult sequential_rollout(
    const SimulationState& state,
    const std::vector<std::size_t>& agent_order,
    const BasePolicy& base_policy,
    RolloutConfig config) {
    RolloutResult result;
    result.controls.resize(state.agents().size());
    for (std::size_t index = 0; index < state.agents().size(); ++index) {
        result.controls[index] = state.agents()[index].position;
    }
    for (std::size_t pos = 0; pos < agent_order.size(); ++pos) {
        const std::size_t selected_index = agent_order[pos];
        double best_cost = std::numeric_limits<double>::infinity();
        Control best_control = state.agents()[selected_index].position;
        const Control current_control = best_control;
        std::vector<Control> candidate_controls = state.environment().legal_moves(best_control);
        candidate_controls.push_back(best_control);
        for (const Control candidate : candidate_controls) {
            std::vector<Control> controls = result.controls;
            controls[selected_index] = candidate;
            for (std::size_t later = pos + 1; later < agent_order.size(); ++later) {
                controls[agent_order[later]] = base_control(
                    state, agent_order[later], base_policy);
            }

            SimulationState future = apply_controls(state, controls);
            double cost = movement_cost(state, future);
            for (int step = 0;
                 step < config.horizon && remaining_known(future, config.known_tasks) > 0;
                 ++step) {
                std::vector<Control> future_controls(future.agents().size());
                for (std::size_t index = 0; index < future_controls.size(); ++index) {
                    future_controls[index] = future.agents()[index].position;
                }
                for (const std::size_t index : agent_order) {
                    future_controls[index] = base_control(future, index, base_policy);
                }
                SimulationState next = apply_controls(future, future_controls);
                cost += movement_cost(future, next);
                future = std::move(next);
            }
            cost += static_cast<double>(config.horizon) *
                static_cast<double>(remaining_known(future, config.known_tasks));
            const bool candidate_moves = candidate != current_control;
            const bool best_moves = best_control != current_control;
            const bool candidate_precedes =
                (candidate_moves && !best_moves) ||
                (candidate_moves == best_moves &&
                 (candidate.row < best_control.row ||
                  (candidate.row == best_control.row && candidate.column < best_control.column)));
            if (cost < best_cost || (cost == best_cost && candidate_precedes)) {
                best_cost = cost;
                best_control = candidate;
            }
        }
        result.controls[selected_index] = best_control;
        result.estimated_cost += best_cost;
    }
    return result;
}

}  // namespace dmar