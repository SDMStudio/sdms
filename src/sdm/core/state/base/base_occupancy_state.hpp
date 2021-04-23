/**
 * @file base_occupancy_state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Base class for occupancy states
 * @version 1.0
 * @date 29/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{

  /**
   * @brief An occupancy state refers to a state in which all the knowledge known by a central planner.
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState, typename TJointHistory_p>
  class BaseOccupancyState : public MappedVector<Pair<TState, TJointHistory_p>, double>
  {
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    BaseOccupancyState();
    BaseOccupancyState(double default_value);
    BaseOccupancyState(std::size_t size, double default_value);
    BaseOccupancyState(const BaseOccupancyState &v);

    /**
     * @brief Get the set of states that are in the support of the occupancy state.
     * 
     * @return the possible states
     */
    std::set<state_type> getStates() const;

    /**
     * @brief Get the set of joint histories that are in the support of the occupancy state.
     * 
     * @return the possible joint hitories
     */
    std::set<jhistory_type> getJointHistories() const;

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
     * 
     */
    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> getAllIndividualHistories() const;

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
     * 
     * @param number the agent id
     */
    std::set<typename jhistory_type::element_type::ihistory_type> getIndividualHistories(number ) const;

    /**
     * @brief Return the state of a precise occupancy state
     */
    TState getState(const Pair<TState, TJointHistory_p> &pair_state_hist) const;

    /**
     * @brief Get the Hidden State of a precise occupancy state. For this situation, the hidden state is the current State
     * 
     * @param pair_state_hist 
     * @return TState 
     */
    TState getHiddenState(const Pair<TState, TJointHistory_p> &pair_state_hist) const;

    /**
     * @brief Return the hidden Joint history of a precise occupancy state
     */
    TJointHistory_p getHistory(const Pair<TState, TJointHistory_p> &pair_state_hist) const;

    /**
     * @brief Return the probability of a precise occupancy state
     */
    double getProbability(const Pair<TState, TJointHistory_p> &pair_state_hist);
  };
} // namespace sdm
#include <sdm/core/state/base/base_occupancy_state.tpp>