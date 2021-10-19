#pragma once

#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>

namespace sdm
{
    class ExhaustiveActionSelection : public ActionSelectionBase
    {
    public:
        using TData = double;

        ExhaustiveActionSelection();
        ExhaustiveActionSelection(const std::shared_ptr<SolvableByHSVI> &world);

        /**
         * @brief Select the best action for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::shared_ptr<Action> : Action
         */
        Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t);
    };
}