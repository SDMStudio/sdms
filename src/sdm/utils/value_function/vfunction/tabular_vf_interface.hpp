#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/value_function/update_rule/vupdate_rule.hpp>

namespace sdm
{
    class TabularValueFunctionInterface : virtual public ValueFunctionInterface
    {
    public:
        TabularValueFunctionInterface(const std::shared_ptr<SolvableByDP> &world,
                                      const std::shared_ptr<Initializer> &initializer = nullptr,
                                      const std::shared_ptr<ActionSelectionInterface> &action = nullptr);

        /**
         * @brief Set the value function at state s and timestep t to a new value.
         *
         * @param state the state
         * @param new_value the new value
         * @param t the timestep
         */
        virtual void setValueAt(const std::shared_ptr<State> &state, double new_value, number t) = 0;
    };
}