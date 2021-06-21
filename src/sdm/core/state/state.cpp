#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/jhistory_interface.hpp>

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

    std::shared_ptr<HistoryInterface> State::toHistoryTree()
    {
        return std::static_pointer_cast<HistoryInterface>(this->shared_from_this());
    }

    std::shared_ptr<JointHistoryInterface> State::toJointHistoryTree()
    {
        return std::static_pointer_cast<JointHistoryInterface>(this->toHistoryTree());
    }

    TypeState State::getTypeState() const 
    {
      return TypeState::STATE;
    }

} // namespace sdm
