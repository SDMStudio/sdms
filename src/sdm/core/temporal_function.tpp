#include <sdm/core/temporal_function.hpp>

namespace sdm
{
    template <typename TFunction>
    TemporalFunction<TFunction>::TemporalFunction(const number horizon = 0) : horizon_(horizon)
    {
        this->temporal_function_ = std::vector<TFunction>((this->horizon_ <= 0) ? 1 : this->horizon_, TFunction());
    }

    template <typename TFunction>
    TFunction &&TemporalFunction<TFunction>::getFunction(const number &t) const
    {
        return this->temporal_function_.at(this->getIndex(t));
    }

    template <typename TFunction>
    const TFunction &TemporalFunction<TFunction>::operator()(const number &t)
    {
        return this->getFunction(t);
    }

    template <typename TFunction>
    void TemporalFunction<TFunction>::setFunction(const TFunction &function, number t) const
    {
        this->temporal_function_[this->getIndex(t)] = function;
    }

    template <typename TFunction>
    number TemporalFunction<TFunction>::getIndex(number t) const
    {
        assert(t < this->horizon_);
        return ((this->horizon_ <= 0) ? 0 : t);
    }
} // namespace sdm
