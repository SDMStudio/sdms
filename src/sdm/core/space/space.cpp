#include <sdm/core/space/space.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>

namespace sdm
{

    bool Space::isContinuous() const
    {
        return !(this->isDiscrete());
    }

    bool Space::operator==(const Space &sp) const
    {
        return (this->str() == sp.str());
    }

    bool Space::operator!=(const Space &sp) const
    {
        return !(this->operator==(sp));
    }

    std::shared_ptr<DiscreteSpace> Space::toDiscreteSpace()
    {
        return std::static_pointer_cast<DiscreteSpace>(this->shared_from_this());
    }

    std::shared_ptr<MultiDiscreteSpace> Space::toMultiDiscreteSpace()
    {
        return std::static_pointer_cast<MultiDiscreteSpace>(this->shared_from_this());
    }

} // namespace sdm
