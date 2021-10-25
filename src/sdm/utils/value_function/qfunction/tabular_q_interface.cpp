#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>

namespace sdm
{
    TabularQValueFunctionInterface::TabularQValueFunctionInterface(number horizon, const std::shared_ptr<Initializer> &initializer,
                                                                   const std::shared_ptr<ActionSelectionInterface> &action_selection)
        : ValueFunctionInterface(horizon, initializer, action_selection)
    {
    }
} // namespace sdm