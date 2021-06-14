#include <sdm/core/state/state.hpp>
#include <sdm/core/state/belief_interface.hpp>

namespace sdm
{

    std::shared_ptr<BeliefInterface> State::toBelief()
    {
        return std::static_pointer_cast<BeliefInterface>(this->shared_from_this());
    }

} // namespace sdm
