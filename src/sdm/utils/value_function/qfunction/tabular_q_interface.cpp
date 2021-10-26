#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>

namespace sdm
{
    TabularQValueFunctionInterface::TabularQValueFunctionInterface(const std::shared_ptr<SolvableByDP> &world,
                                                                   const std::shared_ptr<Initializer> &initializer,
                                                                   const std::shared_ptr<ActionSelectionInterface> &action_selection)
        : ValueFunctionInterface(world, initializer, action_selection)
    {
    }
} // namespace sdm