#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

namespace sdm
{
    class TabularQValueFunctionInterface : virtual public ValueFunctionInterface
    {
    public:
        TabularQValueFunctionInterface(const std::shared_ptr<SolvableByDP> &world,
                                       const std::shared_ptr<Initializer> &intializer = nullptr,
                                       const std::shared_ptr<ActionSelectionInterface> &action_selection = nullptr);

        /**
         * @brief Set the q-value at a specific state s, action a and timestep t.
         *
         * @param state the state
         * @param action the action
         * @param t the time step
         */
        virtual void setQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double value, number t) = 0;
    };
}