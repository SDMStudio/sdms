#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/serialized_state.hpp>

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
    // Faudrait construire d'autre contructeur, et notamment quand on lui donne directement un SerializedState,TjointHistory, et une valeur ?
    
    number getCurrentAgentId() const;
    std::set<typename state_type::state_type> getHiddenStates() const;
    std::set<typename state_type::action_type> getActions() const;

    typename state_type::state_type getHiddenState(const Pair<state_type, jhistory_type> &state) const;
    std::vector<typename state_type::action_type> getAction(const Pair<state_type, jhistory_type> &state) const;
  };
} // namespace sdm
#include <sdm/core/state/serialized_occupancy_state.tpp>

namespace std
{
  template <typename S, typename V>
  struct hash<sdm::SerializedOccupancyState<S, V>>
  {
    typedef sdm::SerializedOccupancyState<S, V> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      size_t seed = 0;
      for (auto &v : in)
      {
        //Combine the hash of the current vector with the hashes of the previous ones
        sdm::hash_combine(seed, v.first);
        sdm::hash_combine(seed, v.second);
      }
      return seed;
    }
  };
}