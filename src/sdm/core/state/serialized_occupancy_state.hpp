#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
  template <typename TState, typename TJointHistory_p>
  class SerializedOccupancyState : public MappedVector<Pair<TState, TJointHistory_p, std::vector<number>>, double>
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

    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> getIndividualHistories() const;
  };
} // namespace sdm
#include <sdm/core/state/occupancy_state.tpp>
