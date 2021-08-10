#include <sdm/utils/value_function/base_value_function.hpp>

namespace sdm
{

    BaseValueFunction::BaseValueFunction() {}

    BaseValueFunction::BaseValueFunction(number horizon) : horizon_(horizon) {}

    number BaseValueFunction::getHorizon() const
    {
        return this->horizon_;
    }

    bool BaseValueFunction::isFiniteHorizon() const
    {
        return (this->horizon_ > 0);
    }

    bool BaseValueFunction::isInfiniteHorizon() const
    {
        return !(this->isFiniteHorizon());
    }

    std::shared_ptr<BaseValueFunction> BaseValueFunction::getptr()
    {
        return this->shared_from_this();
    }
}