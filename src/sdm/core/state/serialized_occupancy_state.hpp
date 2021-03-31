#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
  template <typename TState = number, typename TJointHistory_p = JointHistoryTree_p<number>>
  class SerializedOccupancyState : public MappedVector<Tuple<TState, TJointHistory_p, std::vector<number>>, double>
  {
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    SerializedOccupancyState();
    SerializedOccupancyState(double default_value);
    SerializedOccupancyState(std::size_t size, double default_value);
    SerializedOccupancyState(const SerializedOccupancyState &v);

    std::set<jhistory_type> getJointHistories() const;
    std::set<state_type> getStates() const;

    std::set<typename jhistory_type::element_type::ihistory_type> getIndividualHistories(number ag_id) const;

    number getCurrentAgentId() const;
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