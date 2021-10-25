#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    class State2OccupancyValueFunction : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> mdp_vf_;

    public:
        State2OccupancyValueFunction(std::shared_ptr<ValueFunction>);

        double operator()(const std::shared_ptr<State> &state, number t);

        double operatorState(const std::shared_ptr<State> &state, number t);
        double operatorBeliefState(const std::shared_ptr<BeliefInterface> &belief_state, number t);
        double operatorOccupancyState(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);

        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, number t);
        double operatorQTableState(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, number t);
        double operatorQTableBelief(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, number t);

        bool isPomdpAvailable();
        bool isMdpAvailable();

        std::shared_ptr<ValueFunction> getRelaxation();
        std::shared_ptr<ValueFunction> getMDPValueFunction();
    };
} // namespace sdm