#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    QValueFunction::QValueFunction()
    {
    }

    QValueFunction::QValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                   const std::shared_ptr<Initializer> &initializer,
                                   const std::shared_ptr<ActionSelectionInterface> &action,
                                   const std::shared_ptr<QUpdateRuleInterface> &update_rule)
        : ValueFunctionInterface(world, initializer, action), update_rule_(update_rule)
    {
    }

    double QValueFunction::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->getActionSelection()->getGreedyActionAndValue(this->getptr(), state, t).second;
    }

    void QValueFunction::updateValueAt(double learning_rate, number t)
    {
        this->getUpdateRule()->update(learning_rate, t);
    }

    std::shared_ptr<QUpdateRuleInterface> QValueFunction::getUpdateRule() const
    {
        return this->update_rule_;
    }

    void QValueFunction::setUpdateRule(std::shared_ptr<QUpdateRuleInterface> update_rule)
    {
        this->update_rule_ = update_rule;
    }

    std::shared_ptr<QValueFunction> QValueFunction::getptr()
    {
        return std::dynamic_pointer_cast<QValueFunction>(this->shared_from_this());
    }
} // namespace sdm