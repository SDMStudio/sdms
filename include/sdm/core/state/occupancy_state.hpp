/**
 * @file hsvi.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief HSVI algorithm
 * @version 0.1
 * @date 22/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
  template <typename TState, typename TJointHistory_p>
  class OccupancyState : public MappedVector<Pair<TState, TJointHistory_p>, double>
  {
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    std::set<jhistory_type> getJointHistories();

    std::set<state_type> getStates();

    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> getIndividualHistories();

    // std::string str()
    // {
    //   return "OccupancyMDP";
    // }

    // friend std::ostream &operator<<(std::ostream &os, const OccupancyState &ostate)
    // {
    //   os << ostate.str();
    //   return os;
    // }
  };
} // namespace sdm
#include <sdm/core/state/occupancy_state.tpp>
