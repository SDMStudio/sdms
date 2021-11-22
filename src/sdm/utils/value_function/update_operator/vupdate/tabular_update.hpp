#pragma once

#include <sdm/core/state/state.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

namespace sdm
{
    namespace update
    {
        class TabularUpdate : public TabularUpdateOperator
        {
        public:
            TabularUpdate(const std::shared_ptr<ValueFunctionInterface> &value_function);

            void update(const std::shared_ptr<State> &state, number t);
            void update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
        };
    }
}
