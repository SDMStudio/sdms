#include <sdm/core/space/space.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{

    template <typename TItem>
    bool BaseSpace<TItem>::isContinuous() const
    {
        return !(this->isDiscrete());
    }

    template <typename TItem>
    bool BaseSpace<TItem>::operator==(const BaseSpace &sp) const
    {
        return (this->str() == sp.str());
    }

    template <typename TItem>
    bool BaseSpace<TItem>::operator!=(const BaseSpace &sp) const
    {
        return !(this->operator==(sp));
    }

    template <typename TItem>
    std::shared_ptr<DiscreteSpace<TItem>> BaseSpace<TItem>::toDiscreteSpace()
    {
        return std::static_pointer_cast<DiscreteSpace<TItem>>(this->shared_from_this());
    }

    template <typename TItem>
    std::shared_ptr<MultiDiscreteSpace<TItem>> BaseSpace<TItem>::toMultiDiscreteSpace()
    {
        return std::static_pointer_cast<MultiDiscreteSpace<TItem>>(this->shared_from_this());
    }

} // namespace sdm
