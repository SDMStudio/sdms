#pragma once

#include <sdm/core/function.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    class POMDPRelaxation : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> pomdp_value_function;

    public:
        POMDPRelaxation(std::shared_ptr<ValueFunction> pomdp_vf);

        double operator()(const std::shared_ptr<State> &state, const number &t);
        double getValueAt(const std::shared_ptr<State> &state, const number &t);

        double getValueAtState(const std::shared_ptr<State> &state, const number &t);
        double getValueAtBelief(const std::shared_ptr<BeliefInterface> &belief_state, const number &t);
        double getValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const number &t);
        double getValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const number &t, bool display);

        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, const number &t);
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &t);

        double getQValueAtState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &t);
        double getQValueAtBelief(const std::shared_ptr<BeliefInterface> &belief_state, const std::shared_ptr<Action> &action, const number &t);
        double getQValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &action, const number &t);
        double getQValueAtBelief(const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const number &t, bool display);

        bool isPomdpAvailable();
        bool isMdpAvailable();

        std::shared_ptr<ValueFunction> getRelaxation();
        std::shared_ptr<ValueFunction> getPOMDPValueFunction();
    };
} // namespace sdm
