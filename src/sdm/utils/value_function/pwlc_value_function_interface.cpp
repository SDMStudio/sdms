#include <sdm/utils/value_function/pwlc_value_function_interface.hpp>

namespace sdm
{
    PWLCValueFunctionInterface::PWLCValueFunctionInterface(number horizon, const std::shared_ptr<Initializer> &initializer,
                                                           const std::shared_ptr<ActionSelectionInterface> &action,
                                                           int freq_pruning)
        : ValueFunctionInterface(horizon, initializer, action), PrunableStructure(horizon, freq_pruning)
    {
    }
} // namespace sdm