#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>

namespace sdm
{
    PWLCValueFunctionInterface::PWLCValueFunctionInterface(const std::shared_ptr<SolvableByDP> &world,
                                                           const std::shared_ptr<Initializer> &initializer,
                                                           const std::shared_ptr<ActionSelectionInterface> &action,
                                                           int freq_pruning)
        : ValueFunctionInterface(world, initializer, action), PrunableStructure(world->getHorizon(), freq_pruning)
    {
    }
} // namespace sdm