#include <sdm/core/item.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm
{
    std::shared_ptr<State> Item::toState()
    {
        return std::static_pointer_cast<State>(this->shared_from_this());
    }

    std::shared_ptr<Action> Item::toAction()
    {
        return std::static_pointer_cast<Action>(this->shared_from_this());
    }

    std::shared_ptr<Observation> Item::toObservation()
    {
        return std::static_pointer_cast<Observation>(this->shared_from_this());
    }
} // namespace sdm
