#include <sdm/utils/value_function/vfunction/tabular_vf_interface.hpp>

namespace sdm
{
    TabularValueFunctionInterface::TabularValueFunctionInterface(number horizon, const std::shared_ptr<Initializer> &initializer,
                                                                 const std::shared_ptr<ActionSelectionInterface> &action_selection)
        : ValueFunctionInterface(horizon, initializer, action_selection)
    {
    }
} // namespace sdm