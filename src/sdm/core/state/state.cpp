#include <sdm/core/state/state.hpp>
#include <sdm/core/state/belief_interface.hpp>
#include <sdm/core/state/occupancy_state_interface.hpp>

namespace sdm
{

    std::shared_ptr<BeliefInterface> State::toBelief()
    {
        return std::static_pointer_cast<BeliefInterface>(this->shared_from_this());
    }

    std::shared_ptr<OccupancyStateInterface> State::toOccupancyState()
    {
        return std::static_pointer_cast<OccupancyStateInterface>(this->toBelief());
    }

} // namespace sdm
