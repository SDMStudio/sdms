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
   * @brief A state of occupancy refers to a state in which all the knowledge is known by a central planner and which it relies on to make a decision.
   * 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState = number, typename TJointHistory_p = JointHistoryTree_p<number>>
  class OccupancyState : public MappedVector<Pair<TState, TJointHistory_p>, double>
  {
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    OccupancyState();
    OccupancyState(double default_value);
    OccupancyState(std::size_t size, double default_value);
    OccupancyState(const OccupancyState &v);

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
     * @param ag_id the agent id
     */
    std::set<typename jhistory_type::element_type::ihistory_type> getIndividualHistories(number ag_id) const;

    /**
     * @brief Return the state of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return TState refers to the hidden state returned
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
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return TJointHistory_p refers to the hidden Joint history returned
     */
    TJointHistory_p getHistory(const Pair<TState, TJointHistory_p> &pair_state_hist) const;

    /**
     * @brief Return the probability of a precise occupancy state
     * 
     * @param pair_state_hist refers to a precise occupancy state
     * @return double refers to the probability returned
     */
    double getProbability(const Pair<TState, TJointHistory_p> &pair_state_hist);
  };
} // namespace sdm
#include <sdm/core/state/occupancy_state.tpp>