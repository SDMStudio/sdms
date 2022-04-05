#pragma once

#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/update_rule/vupdate/tabular_update.hpp>

namespace sdm
{
    namespace update
    {
        class LowerBoundTabularUpdate : public TabularUpdate
        {
        public:
            LowerBoundTabularUpdate(const std::shared_ptr<ValueFunctionInterface> &value_function);
            void update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
            void update(const std::shared_ptr<State> &state, double new_value, number t){}
        };
    }
}