#include <sdm/core/state/state.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/interface/history_tree_interface.hpp>
#include <sdm/core/state/interface/jhistory_tree_interface.hpp>

namespace sdm
{

    std::shared_ptr<BeliefInterface> State::toBelief()
    {
        return std::dynamic_pointer_cast<BeliefInterface>(this->shared_from_this());
    }

    std::shared_ptr<OccupancyStateInterface> State::toOccupancyState()
    {
        return std::dynamic_pointer_cast<OccupancyStateInterface>(this->toBelief());
    }

    std::shared_ptr<HistoryTreeInterface> State::toHistoryTree()
    {
        return std::dynamic_pointer_cast<HistoryTreeInterface>(this->shared_from_this());
    }

    std::shared_ptr<JointHistoryTreeInterface> State::toJointHistoryTree()
    {
        return std::dynamic_pointer_cast<JointHistoryTreeInterface>(this->toHistoryTree());
    }

    TypeState State::getTypeState() const 
    {
      return TypeState::State_;
    }

} // namespace sdm
