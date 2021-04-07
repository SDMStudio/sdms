#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue>
    ValueFunction<TState, TAction, TValue>::ValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon) : problem_(problem), horizon_(horizon)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    std::shared_ptr<BinaryFunction<TState, number, TValue>> ValueFunction<TState, TAction, TValue>::getInitFunction()
    {
        return this->init_function_;
    }

    template <typename TState, typename TAction, typename TValue>
    TValue ValueFunction<TState, TAction, TValue>::operator()(const TState &state, const number &t)
    {
        return this->getValueAt(state, t);
    }

    template <typename TState, typename TAction, typename TValue>
    std::shared_ptr<VectorImpl<TAction, TValue>> ValueFunction<TState, TAction, TValue>::getQValueAt(const TState &state, number t)
    {
        std::shared_ptr<MappedVector<TAction, TValue>> q_s = std::make_shared<MappedVector<TAction, TValue>>();
        for (auto &a : this->getWorld()->getActionSpaceAt(state)->getAll())
        {
            (*q_s)[a] = this->getQValueAt(state, a, t);
        }
        return q_s;
    }

    template <typename TState, typename TAction, typename TValue>
    TValue ValueFunction<TState, TAction, TValue>::getQValueAt(const TState &state, const TAction &action, number t)
    {
        // implement bellman operator
        return this->getWorld()->getReward(state, action) + this->getDiscount(t) * this->getWorld()->getExpectedNextValue(this, state, action, t);
    }

    template <typename TState, typename TAction, typename TValue>
    TAction ValueFunction<TState, TAction, TValue>::getBestAction(const TState &state, number t)
    {
        auto qvalues = this->getQValueAt(state, t);
        return qvalues->argmax();
    }

    template <typename TState, typename TAction, typename TValue>
    void ValueFunction<TState, TAction, TValue>::initialize(std::shared_ptr<BinaryFunction<TState, number, TValue>> init_function)
    {
        this->init_function_ = init_function;
    }

    template <typename TState, typename TAction, typename TValue>
    std::shared_ptr<SolvableByHSVI<TState, TAction>> ValueFunction<TState, TAction, TValue>::getWorld()
    {
        return this->problem_;
    }

    template <typename TState, typename TAction, typename TValue>
    int ValueFunction<TState, TAction, TValue>::getHorizon() const
    {
        return this->horizon_;
    }

    template <typename TState, typename TAction, typename TValue>
    bool ValueFunction<TState, TAction, TValue>::isFiniteHorizon() const
    {
        return (this->horizon_ > 0);
    }

    template <typename TState, typename TAction, typename TValue>
    bool ValueFunction<TState, TAction, TValue>::isInfiniteHorizon() const
    {
        return !(this->isFiniteHorizon());
    }

    template <typename TState, typename TAction, typename TValue>
    double ValueFunction<TState, TAction, TValue>::getDiscount(number t)
    {

        if (this->getWorld()->isSerialized())
        {
            if ((t + 1) % this->getWorld()->getUnderlyingProblem()->getNumAgents() != 0)
            {
                return 1.0;
            }
        }
        return this->getWorld()->getUnderlyingProblem()->getDiscount();
    }

} // namespace sdm