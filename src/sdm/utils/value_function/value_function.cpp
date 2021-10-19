#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    using namespace update;

    ValueFunction::ValueFunction() {}

    ValueFunction::ValueFunction(number horizon, const std::shared_ptr<Initializer> &initializer,
                                 const std::shared_ptr<ActionSelectionInterface> &action_selection,
                                 const std::shared_ptr<UpdateOperatorInterface> &update_operator)
        : ValueFunctionInterface(horizon, initializer, action_selection), update_operator_(update_operator)
    {
    }

    ValueFunction::ValueFunction(const ValueFunction &copy) : ValueFunctionInterface(copy.horizon_, copy.initializer_, copy.action_selection_),
                                                              update_operator_(copy.update_operator_)
    {
    }

    ValueFunction::~ValueFunction() {}

    double ValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        return this->getWorld()->getReward(state, action, t) + this->getWorld()->getDiscount(t) * this->getWorld()->getExpectedNextValue(vf, state, action, t);
    }

    double ValueFunction::operator()(const std::shared_ptr<State> &state, const number &t)
    {
        return this->getValueAt(state, t);
    }

    void ValueFunction::initialize()
    {
        ValueFunctionInterface::initialize();
    }

    void ValueFunction::initialize(const std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> &init_function)
    {
        this->init_function_ = init_function;
    }

    std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> ValueFunction::getInitFunction()
    {
        return this->init_function_;
    }

    void ValueFunction::updateValueAt(const std::shared_ptr<State> &state, number t)
    {
        auto [best_action, tmp] = this->getGreedyActionAndValue(state, t);
        this->updateValueAt(state, best_action, t);
    }

    void ValueFunction::updateValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        this->getUpdateOperator()->update(state, action, t);
    }

    std::shared_ptr<UpdateOperatorInterface> ValueFunction::getUpdateOperator() const
    {
        return this->update_operator_;
    }
    void ValueFunction::setUpdateOperator(std::shared_ptr<UpdateOperatorInterface> update_operator)
    {
        this->update_operator_ = update_operator;
    }

    std::shared_ptr<ValueFunction> ValueFunction::getptr()
    {
        return std::static_pointer_cast<ValueFunction>(this->shared_from_this());
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