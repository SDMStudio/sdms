#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    ValueFunction<TState, TAction, TValue, TBackupOperator>::ValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon) : problem_(problem), horizon_(horizon)
    {
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    TValue ValueFunction<TState, TAction, TValue, TBackupOperator>::operator()(const TState &state)
    {
        return this->getValueAt(state);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    std::shared_ptr<VectorImpl<TAction, TValue>> ValueFunction<TState, TAction, TValue, TBackupOperator>::getQValueAt(TState &state, int t)
    {
        std::shared_ptr<MappedVector<TAction, TValue>> q_s = std::make_shared<MappedVector<TAction, TValue>>();
        for (auto &a : this->getWorld()->getActionSpace(state).getAll())
        {
            (*q_s)[a] = this->getQValueAt(state, a, t);
        }
        return q_s;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    TValue ValueFunction<TState, TAction, TValue, TBackupOperator>::getQValueAt(TState &state, TAction &action, int t)
    {
        // implement bellman operator
        return this->getWorld()->getReward(state, action) + this->getWorld()->getDiscount() * this->getWorld()->getExpectedNextValue(this, state, action, t);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    TAction ValueFunction<TState, TAction, TValue, TBackupOperator>::getBestAction(TState &state, int t)
    {
        auto qvalues = this->getQValueAt(state, t);
        return qvalues->argmax();
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    std::shared_ptr<SolvableByHSVI<TState, TAction>> ValueFunction<TState, TAction, TValue, TBackupOperator>::getWorld()
    {
        return this->problem_;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    std::shared_ptr<TBackupOperator<TState, TAction>> ValueFunction<TState, TAction, TValue, TBackupOperator>::getBackupOperator()
    {
        return this->backup_op_;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    int ValueFunction<TState, TAction, TValue, TBackupOperator>::getHorizon() const
    {
        return this->horizon_;
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    bool ValueFunction<TState, TAction, TValue, TBackupOperator>::isFiniteHorizon() const
    {
        return (this->horizon_ > 0);
    }

    template <typename TState, typename TAction, typename TValue, template <typename TI, typename TV> class TBackupOperator>
    bool ValueFunction<TState, TAction, TValue, TBackupOperator>::isInfiniteHorizon() const
    {
        return !(this->isFiniteHorizon());
    }
} // namespace sdm