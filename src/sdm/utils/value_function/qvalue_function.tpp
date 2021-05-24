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
    TValue QValueFunction<TState, TAction, TValue>::getValueAt(const TState &, number)
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction, typename TValue>
    TAction QValueFunction<TState, TAction, TValue>::getBestAction(const TState &state, number t)
    {
        auto qvalues = this->getQValuesAt(state, t);
        return qvalues->argmax();
    }


    template <typename TState, typename TAction, typename TValue>
    std::shared_ptr<QValueFunction<TState, TAction, TValue>> QValueFunction<TState, TAction, TValue>::getptr()
    {
        return std::static_pointer_cast<QValueFunction<TState, TAction, TValue>>(this->shared_from_this());
    }

} // namespace sdm