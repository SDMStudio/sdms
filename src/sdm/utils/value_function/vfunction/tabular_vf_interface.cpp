#include <sdm/utils/value_function/vfunction/tabular_vf_interface.hpp>

namespace sdm
{
    TabularValueFunctionInterface::TabularValueFunctionInterface(number horizon, const std::shared_ptr<Initializer> &initializer,
                                                                 const std::shared_ptr<ActionSelectionInterface> &action,
                                                                 const std::shared_ptr<TabularUpdateOperator> &update_operator)
        : ValueFunction(horizon, initializer, action, update_operator)
    {
    }
} // namespace sdm