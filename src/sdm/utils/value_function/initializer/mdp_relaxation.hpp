#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    class MDPRelaxation : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> mdp_value_function;

    public:
        MDPRelaxation(std::shared_ptr<ValueFunction>);

        double operator()(const std::shared_ptr<State> &state, const number &t);
        double getValueAt(const std::shared_ptr<State> &state, const number &t);

        double getValueAtState(const std::shared_ptr<State> &state, const number &t);
        double getValueAtBelief(const std::shared_ptr<BeliefInterface> &belief_state, const number &t);
        double getValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const number &t);

        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, const number &t);
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> & action, const number &t);
        
        double getQValueAtState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> & action, const number &t);
        double getQValueAtBelief(const std::shared_ptr<BeliefInterface> &belief_state, const std::shared_ptr<Action> & action, const number &t);
        double getQValueAtOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> & action, const number &t);

        bool isPomdpAvailable();
        bool isMdpAvailable();

        std::shared_ptr<ValueFunction> getRelaxation();
        std::shared_ptr<ValueFunction> getMDPValueFunction();
    };
} // namespace sdm