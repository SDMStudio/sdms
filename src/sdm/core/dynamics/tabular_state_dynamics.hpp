/**
 * @file state_dynamics.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 26/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <unordered_set>

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/dynamics/base_state_dynamics.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
  /**
   * @brief This class provide quick accessors to transition probability distributions.
   * 
   */
  class TabularStateDynamics : public BaseStateDynamics
  {
  public:
    using value_type = double;
    using matrix_type = MappedMatrix<std::shared_ptr<State>, std::shared_ptr<State>, value_type>;

    TabularStateDynamics();

    TabularStateDynamics(const TabularStateDynamics &copy);

    virtual ~TabularStateDynamics();

    void initDynamics(number num_states, number num_actions);

    /**
     * @brief    Set the transition probability on one point.
     * @param    x A specific state (the state at timestep t)
     * @param    u A specific (joint) action
     * @param    y A specific state (the state at timestep t+1)
     * @param    double probability of the transition
     * @param    bool whether or not we cumulate probabilities.
     */
    void setTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, double proba, number t = 0, bool cumul = false);

    /**
     * @brief    Get the transition probability on one point.
     * @param    x A specific state (the state at timestep t)
     * @param    u A specific (joint) action
     * @param    y A specific state (the state at timestep t+1)
     * @return   the transition probability
     */
    double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;

    /**
     * @brief    Sets probability transitions
     * @param    t_model matrices of transitions, one for each (joint) action.
     */
    // void setTransitions(const std::unordered_map<std::shared_ptr<Action>, matrix_type> &t_model);

    /**
     * @brief    Sets probability transitions
     * @param    t_model matrices of transitions, one for each (joint) action.
     */
    // std::unordered_map<std::shared_ptr<Action>, matrix_type> getTransitions();

    // /**
    //  * @brief    Returns matrix of probability transitions for the pre-defined action.
    //  * @param    u A specific (joint) action
    //  */
    // const matrix_type &getTransitions(const std::shared_ptr<Action> &action);

    /**
     * @brief Get the list of all reachable states 
     * 
     * @param state the current state
     * @param action the current action
     * @return the list of accessible states 
     */
    std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

  protected:
    /** @brief transition and observation matrices */
    std::unordered_map<std::shared_ptr<Action>, matrix_type> t_model;

    /** @brief map from state, action pairs to set of next states */
    std::unordered_map<std::shared_ptr<State>, std::unordered_map<std::shared_ptr<Action>, std::set<std::shared_ptr<State>>>> successor_states;
  };

} // namespace sdm
