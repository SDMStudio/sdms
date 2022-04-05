#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    using namespace update;

    ValueFunction::ValueFunction() {}

    ValueFunction::ValueFunction(const std::shared_ptr<SolvableByDP> &world,
                                 const std::shared_ptr<Initializer> &initializer,
                                 const std::shared_ptr<ActionSelectionInterface> &action_selection)
        : ValueFunctionInterface(world, initializer, action_selection)
    {
    }

    ValueFunction::ValueFunction(const ValueFunction &copy)
        : ValueFunction(copy.world_, copy.initializer_, copy.action_selection_)
    {
        this->update_rule_ = copy.update_rule_;
    }

    double ValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getWorld()->getReward(state, action, t) + this->getWorld()->getDiscount(t) * this->getWorld()->getExpectedNextValue(this->getptr(), state, action, t);
    }

    double ValueFunction::operator()(const std::shared_ptr<State> &state, const number &t)
    {
        return this->getValueAt(state, t);
    }

    std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> ValueFunction::getInitFunction()
    {
        return this->init_function_;
    }

    void ValueFunction::setInitFunction(const std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> &init_function)
    {
        this->init_function_ = init_function;
    }

    void ValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        this->getUpdateRule()->update(state, t);
    }

    std::shared_ptr<UpdateRuleInterface> ValueFunction::getUpdateRule() const
    {
        return this->update_rule_;
    }
    void ValueFunction::setUpdateRule(std::shared_ptr<UpdateRuleInterface> update_rule)
    {
        this->update_rule_ = update_rule;
    }

    std::shared_ptr<ValueFunction> ValueFunction::getptr()
    {
        return std::dynamic_pointer_cast<ValueFunction>(this->shared_from_this());
    }

    size_t ValueFunction::getSize() const
    {
        size_t size_total = 0;
        for (number t = 0; t < this->getHorizon(); t++)
        {
            size_total += this->getSize(t);
        }
        return size_total;
    }
} // namespace sdm