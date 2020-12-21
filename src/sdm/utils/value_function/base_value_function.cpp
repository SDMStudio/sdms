
#include <sdm/utils/value_function/base_value_function.hpp>

namespace sdm
{
    BaseValueFunction::BaseValueFunction(TStruct init_values) : TStruct(init_values)
    {
    }

    vType BaseValueFunction::getValueAt(TState state) const
    {
        return this->operator()(state);
    }

    vType PWLCValueFunction::getValueAt(TState state) const
    {
        vType current, max = std::numeric_limits<vType>::min();

        vType current, min = std::numeric_limits<vType>::max();

        for (const auto &entry : this->container.at(h))
        {
            min = std::min(min, this->getSawtooth(frequency, entry));
        }

        return mdp + min;
    }
    for (const auto &entry : this->data_.container)
    {
        if (max < (current = ((*entry.second) ^ (*frequency))))
            max = current;
    }
    return max;
} // namespace sdm

void BaseValueFunction::updateValueAt(TState state, vType value)
{
}
} // namespace sdm