#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue>
    ValueFunction<TState, TAction, TValue>::ValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, number horizon) : BaseValueFunction<TState, TAction, TValue>(horizon), problem_(problem)
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
        // Compute Q(s,*)
        std::shared_ptr<MappedVector<TAction, TValue>> q_s = std::make_shared<MappedVector<TAction, TValue>>();
        for (const auto &a : this->getWorld()->getActionSpaceAt(state)->getAll())
        {
            (*q_s)[a] = this->getQValueAt(state, a, t);
        }
        return q_s;
    }

    template <typename TState, typename TAction, typename TValue>
    TValue ValueFunction<TState, TAction, TValue>::getQValueAt(const TState &state, const TAction &action, number t)
    {
        // implement bellman operator
        return this->getWorld()->getReward(state, action) + this->getDiscount(t) * this->getWorld()->getExpectedNextValue(this->getptr(), state, action, t);
    }

    template <typename TState, typename TAction, typename TValue>
    TAction ValueFunction<TState, TAction, TValue>::getBestAction(const TState &state, number t)
    {
        // Get the best action (i.e. the action that maximizes the q value function)
        return this->getQValueAt(state, t)->argmax();
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


    template <typename TState, typename TAction, typename TValue>
    std::shared_ptr<ValueFunction<TState, TAction, TValue>> ValueFunction<TState, TAction, TValue>::getptr()
    {
        return std::static_pointer_cast<ValueFunction<TState, TAction, TValue>>(this->shared_from_this());
    }

    template <typename TState, typename TAction, typename TValue>
    size_t ValueFunction<TState, TAction, TValue>::getSize() const
    {
        size_t size_total = 0;

        for(number t= 0;t<this->horizon_;t++)
        {
            size_total += this->getSize(t);
        }
        return size_total;
    }

} // namespace sdm