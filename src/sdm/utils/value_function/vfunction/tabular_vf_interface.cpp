#include <sdm/utils/value_function/vfunction/tabular_vf_interface.hpp>

namespace sdm
{
    TabularValueFunctionInterface::TabularValueFunctionInterface(const std::shared_ptr<SolvableByDP> &world,
                                                                 const std::shared_ptr<Initializer> &initializer,
                                                                 const std::shared_ptr<ActionSelectionInterface> &action_selection)
        : ValueFunctionInterface( world, initializer, action_selection)
    {
    }
} // namespace sdm