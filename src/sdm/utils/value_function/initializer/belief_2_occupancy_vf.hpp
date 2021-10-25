#pragma once

#include <sdm/core/function.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    class Belief2OccupancyValueFunction : public RelaxedValueFunction
    {
    protected:
        std::shared_ptr<ValueFunction> pomdp_value_function;

    public:
        Belief2OccupancyValueFunction(std::shared_ptr<ValueFunction> pomdp_vf);

        double operator()(const std::shared_ptr<State> &state, number t);
        double operator()(const Pair<std::shared_ptr<State>, std::shared_ptr<Action>> &state_AND_action, number t);

        double operatorState(const std::shared_ptr<State> &state, number t);
        double operatorBeliefState(const std::shared_ptr<BeliefInterface> &belief_state, number t);
        double operatorOccupancyState(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);

        bool isPomdpAvailable();
        bool isMdpAvailable();

        std::shared_ptr<ValueFunction> getRelaxation();
        std::shared_ptr<ValueFunction> getPOMDPValueFunction();
    };
} // namespace sdm
