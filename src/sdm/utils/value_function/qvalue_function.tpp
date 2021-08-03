#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{

    template <class TInput>
    QValueFunction<TInput>::QValueFunction()
    {
    }

    template <class TInput>
    QValueFunction<TInput>::QValueFunction(number horizon) : BaseValueFunction(horizon)
    {
    }

    template <class TInput>
    std::shared_ptr<QValueFunction<TInput>> QValueFunction<TInput>::getptr()
    {
        return std::static_pointer_cast<QValueFunction>(this->shared_from_this());
    }

} // namespace sdm