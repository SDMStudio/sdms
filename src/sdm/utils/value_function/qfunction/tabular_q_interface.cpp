#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

namespace sdm
{
    TabularQValueFunctionInterface::TabularQValueFunctionInterface(number horizon,
                                                                   const std::shared_ptr<Initializer> &initializer,
                                                                   const std::shared_ptr<ActionSelectionInterface> &action,
                                                                   const std::shared_ptr<TabularQUpdateOperator> &update_operator)
        : QValueFunction(horizon, initializer, action, update_operator)
    {
    }
} // namespace sdm