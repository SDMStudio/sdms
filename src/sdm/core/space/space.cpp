#include <sdm/core/space/space.hpp>

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

} // namespace sdm
