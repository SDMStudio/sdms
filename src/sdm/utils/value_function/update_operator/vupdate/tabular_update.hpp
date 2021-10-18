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
            TabularUpdate(const std::shared_ptr<SolvableByHSVI> &world, const std::shared_ptr<TabularValueFunctionInterface> &value_function);

            void update(std::shared_ptr<State> state, std::shared_ptr<Action> action, number t);
        };
    }
}
