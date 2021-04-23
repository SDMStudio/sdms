/**
 * @file occupancy_state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 29/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <string>

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/history.hpp>

namespace sdm
{

  /**
   * @brief A private state of occupancy refers to a 
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState = number, typename TJointHistory_p = JointHistoryTree_p<number>>
  class PrivateOccupancyState : public BaseOccupancyState<TState, TJointHistory_p>
  {
  public:
    using jhistory_type = typename BaseOccupancyState<TState, TJointHistory_p>::jhistory_type;
    using state_type = typename BaseOccupancyState<TState, TJointHistory_p>::state_type;

    PrivateOccupancyState();
    PrivateOccupancyState(double default_value);
    PrivateOccupancyState(std::size_t size, double default_value);
    PrivateOccupancyState(const PrivateOccupancyState &v);
  };
} // namespace sdm
#include <sdm/core/state/private_occupancy_state.tpp>
