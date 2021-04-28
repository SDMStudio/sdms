#include <sdm/utils/value_function/base_value_function.hpp>

namespace sdm
{

    template <typename TState, typename TAction, typename TValue>
    BaseValueFunction<TState, TAction, TValue>::BaseValueFunction() {}

    template <typename TState, typename TAction, typename TValue>
    BaseValueFunction<TState, TAction, TValue>::BaseValueFunction(number horizon) : horizon_(horizon) {}

    template <typename TState, typename TAction, typename TValue>
    number BaseValueFunction<TState, TAction, TValue>::getHorizon() const
    {
        return this->horizon_;
    }

    template <typename TState, typename TAction, typename TValue>
    bool BaseValueFunction<TState, TAction, TValue>::isFiniteHorizon() const
    {
        return (this->horizon_ > 0);
    }

    template <typename TState, typename TAction, typename TValue>
    bool BaseValueFunction<TState, TAction, TValue>::isInfiniteHorizon() const
    {
        return !(this->isFiniteHorizon());
    }

    template <typename TState, typename TAction, typename TValue>
    std::shared_ptr<BaseValueFunction<TState, TAction, TValue>> BaseValueFunction<TState, TAction, TValue>::getptr()
    {
        return this->shared_from_this();
    }

}