#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/state/serialized_state.hpp>
#include <sdm/core/state/history.hpp>

namespace sdm
{
  /**
   * @brief A serialized occupancy state refers to an occupancy state (i.e refers to the whole knowledge that a central planner can have access to take decisions) with a precise state for each agent. 
   * But in this implementation we call serialized occupancy state a distribution over serialized state and joint histories .
   * 
   * @tparam TState
   * @tparam TJointHistory_p 
   */
  template <typename TState = SerializedState, typename TJointHistory_p = JointHistoryTree_p<number>>
  class SerializedOccupancyState : public OccupancyState<TState, TJointHistory_p>
  {
  public:
    using jhistory_type = typename OccupancyState<TState, TJointHistory_p>::jhistory_type;
    using state_type = typename OccupancyState<TState, TJointHistory_p>::state_type;

    SerializedOccupancyState();
    SerializedOccupancyState(double default_value);
    SerializedOccupancyState(std::size_t size, double default_value);
    SerializedOccupancyState(const SerializedOccupancyState &v);
    SerializedOccupancyState(const OccupancyState<TState, TJointHistory_p> &v);

    number getCurrentAgentId() const;
    std::set<typename state_type::state_type> getHiddenStates() const;
    std::set<typename state_type::action_type> getActions() const;

    typename state_type::state_type getHiddenState(const Pair<state_type, jhistory_type> &state) const;
    std::vector<typename state_type::action_type> getAction(const Pair<state_type, jhistory_type> &state) const;
    std::shared_ptr<SerializedOccupancyState<TState, TJointHistory_p>> getptr();
  };
} // namespace sdm
#include <sdm/core/state/serialized_occupancy_state.tpp>