#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    QValueFunction::QValueFunction()
    {
    }

    QValueFunction::QValueFunction(number horizon,
                                   const std::shared_ptr<Initializer> &intializer,
                                   const std::shared_ptr<ActionSelectionInterface> &action,
                                   const std::shared_ptr<UpdateOperatorInterface> &update_operator)
        : ValueFunctionInterface(horizon, intializer, action, update_operator)
    {
    }

    std::shared_ptr<QValueFunction> QValueFunction::getptr()
    {
        return std::static_pointer_cast<QValueFunction>(this->shared_from_this());
    }

    double QValueFunction::getValueAt(const std::shared_ptr<State> &state, number t = 0)
    {
        return this->getActionSelection()->getGreedyAction().second;
    }
} // namespace sdm