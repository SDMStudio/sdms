#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TValue>
    QValueFunction<TState, TAction, TValue>::QValueFunction()
    {
    }

    template <typename TState, typename TAction, typename TValue>
    QValueFunction<TState, TAction, TValue>::QValueFunction(number horizon) : BaseValueFunction<TState, TAction, TValue>(horizon)
    {
    }

    template <typename TState, typename TAction, typename TValue>
    TValue QValueFunction<TState, TAction, TValue>::getValueAt(const TState &state, number t)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue>
    TAction QValueFunction<TState, TAction, TValue>::getBestAction(const TState &state, number t)
    {
        auto qvalues = this->getQValueAt(state, t);
        return qvalues->argmax();
    }

} // namespace sdm