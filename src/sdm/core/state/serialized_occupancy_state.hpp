#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{
  template <typename TState = SerializedState<number,number>, typename TJointHistory_p = JointHistoryTree_p<number>>
  class SerializedOccupancyState : public OccupancyState<TState,TJointHistory_p>
  {
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    SerializedOccupancyState();
    SerializedOccupancyState(double default_value);
    SerializedOccupancyState(std::size_t size, double default_value);
    SerializedOccupancyState(const SerializedOccupancyState &v);
    // Faudrait construire d'autre contructeur, et notamment quand on lui donne directement un SerializedState,TjointHistory, et une valeur ? 

    //std::set<jhistory_type> getJointHistories() const;
    //std::set<state_type> getStates() const;

    //std::set<typename jhistory_type::element_type::ihistory_type> getIndividualHistories(number ag_id) const;

    number getCurrentAgentId() const;
    std::set<typename state_type::state_type> getHiddenStates() const;
    std::set<typename state_type::action_type> getActions() const;

    typename state_type::state_type getHiddenState(const Pair<state_type,jhistory_type> &state) const;
    std::vector<typename state_type::action_type> getAction(const Pair<state_type,jhistory_type> &state) const;
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