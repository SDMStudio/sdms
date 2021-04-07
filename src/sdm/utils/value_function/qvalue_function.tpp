#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils//value_function/qvalue_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction, typename TValue>
    QValueFunction<TState, TAction, TValue>::QValueFunction(int horizon) : horizon_(horizon)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TAction QValueFunction<TState, TAction, TValue>::getBestAction(const TState &state, int t)
    {
        auto qvalues = this->getQValueAt(state, t);
        return qvalues->argmax();
    }

    template <typename TState, typename TAction, typename TValue>
    int QValueFunction<TState, TAction, TValue>::getHorizon() const
    {
        return this->horizon_;
    }

    template <typename TState, typename TAction, typename TValue>
    bool QValueFunction<TState, TAction, TValue>::isFiniteHorizon() const
    {
        return (this->horizon_ > 0);
    }

    template <typename TState, typename TAction, typename TValue>
    bool QValueFunction<TState, TAction, TValue>::isInfiniteHorizon() const
    {
        return !(this->isFiniteHorizon());
    }

} // namespace sdm