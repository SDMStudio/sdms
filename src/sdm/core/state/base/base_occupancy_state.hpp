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
  public:
    using jhistory_type = TJointHistory_p;
    using state_type = TState;

    BaseOccupancyState();
    BaseOccupancyState(double);
    BaseOccupancyState(std::size_t, double);
    BaseOccupancyState(const BaseOccupancyState &);

    void setProbabilityAt(const TState &state, const TJointHistory_p &jhist, double proba);
    void setProbabilityAt(const Pair<TState, TJointHistory_p> &pair_state_hist, double proba);

    void addProbabilityAt(const TState &state, const TJointHistory_p &jhist, double proba);
    void addProbabilityAt(const Pair<TState, TJointHistory_p> &pair_state_hist, double proba);

    void finalize();

    /**
     * @brief Get the set of states that are in the support of the occupancy state.
     * @comment: Very bad idea, since you may end of with too many states wrt each joint histories. Instead we should get back the belief state according with a given joint history.
     *            -- require run of setJointHistories(); 
     * @return the possible states
     */
    const std::set<state_type> &getStates() const;

    void setStates();

    /**
     * @brief Get the set of joint histories that are in the support of the occupancy state.
     * @comment: Should be pre-computed -- require run of setJointHistories();
     * @return the possible joint hitories
     */
    const std::set<jhistory_type> &getJointHistories() const;

    void setJointHistories();

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
     * @comment: Should be pre-computed,  -- require run of setAllIndividualHistories();
     */
    const std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> &getAllIndividualHistories() const;
    void setAllIndividualHistories();

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
     * @comment: Should be pre-computed 
     * @param number the agent identifier
     */
    const std::set<typename jhistory_type::element_type::ihistory_type> &getIndividualHistories(number ag_id) const;

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
    double getProbability(const Pair<TState, TJointHistory_p> &pair_state_hist);

    std::string str() const;

    /**
     *  @brief  Returns an ostream instance
     *
     *  This method allows the \@occupancy_state instances to be printed
     *  using an XML style, e.g., for the dpomdp-tiger problem, we'll have
     *  <occupation-state  horizon="3">
     *      <joint-history id="0"  belief="0.7225 0.175">
     *        <agent id="1" history="hear-left:hear-left::hear-left"/>
     *        <agent id="0" history="hear-right:hear-left::hear-left"/>
     *      </joint-history>
     *  </occupation-state>
     *  This method requires \@dpomdp havind a semantic associated with each
     *  action and observation id. If no semantic is available, it print the id.
     */
    friend std::ostream &operator<<(std::ostream &os, BaseOccupancyState &ostate)
    {
      os << ostate.str();
      return os;
    }

  protected:
    /**
     * @brief space of all reachable states, those in the support of the occupancy state
     * @comment: Should not be used since there are to much possible wrt each joint history, belief states whould have been a better choice.
     */
    std::set<state_type> list_states;

    /**
     * @brief space of joint histories of all agents
     */
    std::set<jhistory_type> list_jhistories;

    /**
     * @brief tuple of private history spaces, one private history space per agent
     */
    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> all_list_ihistories;
  };
} // namespace sdm
#include <sdm/core/state/base/base_occupancy_state.tpp>