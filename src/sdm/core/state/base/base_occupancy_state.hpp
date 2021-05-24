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
#include <sdm/core/state/beliefs.hpp>
#include <sdm/core/state/history.hpp>

namespace sdm
{

  /**
   * @brief An occupancy state refers to a state in which all the knowledge known by a central planner.
   * @comment: Instead of state-history pairs, I'd suggest using belief-history pairs for the sake of efficiency. 
   * @tparam TState refers to a number
   * @tparam TJointHistory_p refers to a joint histories
   */
  template <typename TState, typename TJointHistory_p>
  class BaseOccupancyState : public MappedVector<Pair<TState, TJointHistory_p>, double>,
                             public std::enable_shared_from_this<BaseOccupancyState<TState, TJointHistory_p>>
  {
  public:
    static double PRECISION;

    using state_type = TState;
    using jhistory_type = TJointHistory_p;
    using struct_type = MappedVector<Pair<TState, TJointHistory_p>, double>;

    BaseOccupancyState(double default_value);
    BaseOccupancyState(number num_agents = 2, double default_value = 0.);
    BaseOccupancyState(const BaseOccupancyState &);

    void setProbabilityAt(const TState &, const TJointHistory_p &, double);
    void setProbabilityAt(const Pair<TState, TJointHistory_p> &, double);

    void addProbabilityAt(const TState &, const TJointHistory_p &, double);
    void addProbabilityAt(const Pair<TState, TJointHistory_p> &, double);

    void finalize();

    /**
     * @brief Get the set of states that are in the support of the occupancy state.
     * @comment: Very bad idea, since you may end of with too many states wrt each joint histories. Instead we should get back the belief state according with a given joint history.
     *            -- require run of setJointHistories(); 
     * @return the possible states per joint histories
     */
    const std::set<TState> &getStates() const;
    const std::set<TState> &getStatesAt(const TJointHistory_p &) const;
    void setStates();

    /**
     * @brief Get the set of joint histories that are in the support of the occupancy state.
     * @comment: Should be pre-computed -- require run of setJointHistories();
     * @return the possible joint hitories
     */
    const std::set<jhistory_type> &getJointHistories() const;
    void setJointHistories();

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for a given agent).
     * @comment: Should be pre-computed 
     * @param number the agent identifier
     */
    const std::set<typename jhistory_type::element_type::ihistory_type> &getIndividualHistories(number) const;

    /**
     * @brief Get the set of individual histories that are in the support of the occupancy state (for all agents).
     * @comment: Should be pre-computed,  -- require run of setAllIndividualHistories();
     */
    const std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> &getAllIndividualHistories() const;
    void setAllIndividualHistories();

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
    double getProbability(const Pair<TState, TJointHistory_p> &) const;

    /**
     * @brief Get all Joint History associated with a precise agent and an individual history
     * 
     * @param number :Agent Id
     * @param typename jhistory_type::element_type::ihistory_type : Individual History
     */
    const std::unordered_set<jhistory_type> &getJointHistoryOverIndividualHistories(number, typename jhistory_type::element_type::ihistory_type) const;
    void setJointHistoryOverIndividualHistories();

    /**
     * @brief Get all the probability conditionning to a Joint History
     * 
     * @param jhistory_type : Joint History
     */
    const double &getProbabilityOverJointHistory(jhistory_type) const;
    void setProbabilityOverJointHistory();

    /**
     * @brief Return a shared pointer on current object
     */
    std::shared_ptr<BaseOccupancyState<TState, TJointHistory_p>> getptr();

    std::string str() const;
    std::string str_hyperplan() const;

    bool operator==(const BaseOccupancyState<TState, TJointHistory_p> &other) const;

    /**
     *  @brief  Returns an ostream instance
     */
    friend std::ostream &operator<<(std::ostream &os, BaseOccupancyState<TState, TJointHistory_p> &ostate)
    {
      os << ostate.str();
      return os;
    }

  protected:
    /**
     * @brief the number of agents 
     */
    number num_agents_ = 2;

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
     * @brief space of joint history and state of all agents
     */
    RecursiveMap<jhistory_type, std::set<state_type>> list_jhistory_states;

    /**
     * @brief tuple of private history spaces, one private history space per agent
     */
    std::vector<std::set<typename jhistory_type::element_type::ihistory_type>> all_list_ihistories;

    std::unordered_map<number, std::unordered_map<typename jhistory_type::element_type::ihistory_type, std::unordered_set<jhistory_type>>> ihistories_to_jhistory;

    RecursiveMap<jhistory_type, double> probability_jhistories;
  };

} // namespace sdm
#include <sdm/core/state/base/base_occupancy_state.tpp>

namespace std
{
  template <typename S, typename V>
  struct hash<sdm::BaseOccupancyState<S, V>>
  {
    typedef sdm::BaseOccupancyState<S, V> argument_type;
    typedef std::size_t result_type;
    inline result_type operator()(const argument_type &in) const
    {
      return std::hash<sdm::MappedVector<sdm::Pair<S, V>, double>>()(in);
    }
  };
}