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
   * @comment: Instead of state-history pairs, I'd suggest using belief-history pairs for the sake of efficiency. 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState, typename TJointHistory_p>
  class BaseOccupancyState : public MappedVector<Pair<TState, TJointHistory_p>, double>
  {
  protected:
    /**
     * @brief tuple of private history spaces, one private history space per agent
     */
    std::vector<std::set<typename TJointHistory_p::element_type::ihistory_type>> agent_history_spaces;

    /**
     * @brief space of joint histories of all agents
     */
    std::set<TJointHistory_p> joint_history_space;

    /**
     * @brief space of all reachable states, those in the support of the occupancy state
     * @comment: Should not be used since there are to much possible wrt each joint history, belief states whould have been a better choice.
     */
    std::set<TState> state_space;

  public:
  using jhistory_type = TJointHistory_p;
  using state_type = TState;

    BaseOccupancyState();
    BaseOccupancyState(double);
    BaseOccupancyState(std::size_t, double);
    BaseOccupancyState(const BaseOccupancyState &);

    /**
     * @brief Get the set of states that are in the support of the occupancy state.
     * @comment: Very bad idea, since you may end of with too many states wrt each joint histories. Instead we should get back the belief state according with a given joint history.
     *            -- require run of setJointHistories(); 
     * @return the possible states
     */
    std::set<state_type> getStates() const;
    void setStates();

    /**
     * @brief Get the set of joint histories that are in the support of the occupancy state.
     * @comment: Should be pre-computed -- require run of setJointHistories();
     * @return the possible joint hitories
     */
    std::set<jhistory_type> getJointHistories() const;
    void setJointHistories();

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
     * @comment: Should be pre-computed,  -- require run of setAllIndividualHistories();
     */
    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> getAllIndividualHistories() const;
    void setAllIndividualHistories();

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
     * @comment: Should be pre-computed 
     * @param number the agent identifier
     */
    std::set<typename jhistory_type::element_type::ihistory_type> getIndividualHistories(number) const;

    /**
     * @brief Return the state of a precise occupancy state
     * @comment: what is the difference with getHiddenState()
     */
    TState getState(const Pair<TState, TJointHistory_p> &) const;

    /**
     * @brief Get the Hidden State of a precise occupancy state. For this situation, the hidden state is the current State
     * @comment: what is the difference with getState()
     * @param pair of state and joint history 
     * @return TState 
     */
    TState getHiddenState(const Pair<TState, TJointHistory_p> &) const;

    /**
     * @brief Return the hidden Joint history of a precise occupancy state
     */
    TJointHistory_p getHistory(const Pair<TState, TJointHistory_p> &) const;

    /**
     * @brief Return the probability of a precise occupancy state
     */
    double getProbability(const Pair<TState, TJointHistory_p> &);
  };
} // namespace sdm
#include <sdm/core/state/base/base_occupancy_state.tpp>