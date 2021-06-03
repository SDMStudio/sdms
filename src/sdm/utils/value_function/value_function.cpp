#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    ValueFunction::ValueFunction(const std::shared_ptr<SolvableByHSVI> &problem, number horizon) : BaseValueFunction(horizon), problem_(problem)
    {
    }

    std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> ValueFunction::getInitFunction()
    {
        return this->init_function_;
    }

    double ValueFunction::operator()(const std::shared_ptr<State> &state, const number &t)
    {
        return this->getValueAt(state, t);
    }

    std::shared_ptr<VectorImpl<std::shared_ptr<Action>, double>> ValueFunction::getQValueAt(const std::shared_ptr<State> &state, number t)
    {
        // Compute Q(s,*)
        std::shared_ptr<MappedVector<std::shared_ptr<Action>, double>> q_s = std::make_shared<MappedVector<std::shared_ptr<Action>, double>>();
        for (const auto &a : this->getWorld()->getActionSpaceAt(state, t)->getAll())
        {
            (*q_s)[a] = this->getQValueAt(state, a, t);
        }
        return q_s;
    }

    double ValueFunction::getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
    {
        // implement bellman operator
        return this->getWorld()->getReward(state, action, t) + this->getWorld()->getDiscount(t) * this->getWorld()->getExpectedNextValue(this->getptr(), state, action, t);
    }

    std::shared_ptr<Action> ValueFunction::getBestAction(const std::shared_ptr<State> &state, number t)
    {
        // Get the best action (i.e. the action that maximizes the q value function)
        return this->getQValueAt(state, t)->argmax();
    }

    void ValueFunction::initialize(std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> init_function)
    {
        this->init_function_ = init_function;
    }

    std::shared_ptr<SolvableByHSVI> ValueFunction::getWorld()
    {
        return this->problem_;
    }
    
    std::shared_ptr<ValueFunction> ValueFunction::getptr()
    {
        return std::static_pointer_cast<ValueFunction>(this->shared_from_this());
    }

} // namespace sdm