#include <sdm/utils/value_function/vfunction/pwlc_vf_interface.hpp>

namespace sdm
{
    PWLCValueFunctionInterface::PWLCValueFunctionInterface(number horizon, const std::shared_ptr<Initializer> &initializer,
                                                           const std::shared_ptr<ActionSelectionInterface> &action,
                                                           const std::shared_ptr<PWLCUpdateOperator> &update_operator,
                                                           int freq_pruning = -1)
        : ValueFunctionApproximationInterface(horizon, initializer, action, update_operator, freq_pruning)
    {
    }
} // namespace sdm