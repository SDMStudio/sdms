#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue>
    ValueFunction<TState, TAction, TValue>::ValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon,int extensive_agent) : problem_(problem), horizon_(horizon),extensive_agent_(extensive_agent)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TValue ValueFunction<TState, TAction, TValue>::operator()(const TState &state)
    {
        return this->getValueAt(state);
    }

    template <typename TState, typename TAction, typename TValue>
    std::shared_ptr<VectorImpl<TAction, TValue>> ValueFunction<TState, TAction, TValue>::getQValueAt(const TState &state, int t)
    {
        std::shared_ptr<MappedVector<TAction, TValue>> q_s = std::make_shared<MappedVector<TAction, TValue>>();
        for (auto &a : this->getWorld()->getActionSpaceAt(state)->getAll())
        {
            (*q_s)[a] = this->getQValueAt(state, a, t);
        }
        return q_s;
    }

    template <typename TState, typename TAction, typename TValue>
    TValue ValueFunction<TState, TAction, TValue>::getQValueAt(const TState &state, const TAction &action, int t)
    {
        // implement bellman operator
        return this->getWorld()->getReward(state, action) + this->getDiscount(t)* this->getWorld()->getExpectedNextValue(this, state, action, t);
    }

    template <typename TState, typename TAction, typename TValue>
    TAction ValueFunction<TState, TAction, TValue>::getBestAction(const TState &state, int t)
    {
        auto qvalues = this->getQValueAt(state, t);
        return qvalues->argmax();
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
    double ValueFunction<TState, TAction, TValue>::getDiscount(int t)
    {
        
        if(this->extensive_agent_ >1)
        {
            if(t%this->extensive_agent_ != this->extensive_agent_ -1)
            {
                return 1.0;
            }
        }

        return this->getWorld()->getDiscount();
    }


} // namespace sdm