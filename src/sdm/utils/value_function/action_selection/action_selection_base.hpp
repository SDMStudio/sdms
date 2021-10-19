#pragma once

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>
#include <sdm/utils/value_function/action_selection/action_selection_interface.hpp>

namespace sdm
{
    class ActionSelectionBase : public ActionSelectionInterface
    {
    public:
        ActionSelectionBase();
        ActionSelectionBase(const std::shared_ptr<SolvableByHSVI> &world);
        virtual ~ActionSelectionBase();

        /**
         * @brief Select the best action for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::shared_ptr<Action> : Action
         */
        Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t) = 0;

        std::shared_ptr<SolvableByHSVI> getWorld() const;

    protected:
        std::shared_ptr<SolvableByHSVI> world_;
    };
}
