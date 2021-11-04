#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    QValueFunction::QValueFunction()
    {
    }

    QValueFunction::QValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                   const std::shared_ptr<Initializer> &initializer,
                                   const std::shared_ptr<ActionSelectionInterface> &action,
                                   const std::shared_ptr<QUpdateOperatorInterface> &update_operator)
        : ValueFunctionInterface(world, initializer, action), update_operator_(update_operator)
    {
    }

    double QValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->getActionSelection()->getGreedyActionAndValue(this->getptr(), state, t).second;
    }

    void QValueFunction::updateValueAt(number t)
    {
        this->getUpdateOperator()->update(t);
    }

    std::shared_ptr<QUpdateOperatorInterface> QValueFunction::getUpdateOperator() const
    {
        return this->update_operator_;
    }

    void QValueFunction::setUpdateOperator(std::shared_ptr<QUpdateOperatorInterface> update_operator)
    {
        this->update_operator_ = update_operator;
    }

    std::shared_ptr<QValueFunction> QValueFunction::getptr()
    {
        return std::dynamic_pointer_cast<QValueFunction>(this->shared_from_this());
    }
} // namespace sdm