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
#include <sdm/utils/linear_algebra/matrix.hpp>

namespace sdm
{
  /**
   * @brief This class provide quick accessors to transition probability distributions.
   * 
   */
  class StateDynamics
  {
  public:
    StateDynamics();
    StateDynamics(StateDynamics &copy);

    /**
     * @brief   Construct a new State Dynamics object
     * @param   num_actions Number of (joint) action
     * @param   num_states Number of states
     * 
     */
    StateDynamics(number num_actions, number num_states);

    /**
     * @brief   Initialize the dynamics model
     * @param   num_actions Number of (joint) actions
     * @param   num_states Number of states
     */
    void initDynamics(number num_actions, number num_states);

    /**
     * @brief    Set the transition probability on one point.
     * @param    x A specific state (the state at timestep t)
     * @param    u A specific (joint) action
     * @param    y A specific state (the state at timestep t+1)
     * @param    double probability of the transition
     * @param    bool whether or not we cumulate probabilities.
     */
    void setTransitionProbability(number x, number u, number y, double p, bool cumul = false);

    /**
     * @brief    Get the transition probability on one point.
     * @param    x A specific state (the state at timestep t)
     * @param    u A specific (joint) action
     * @param    y A specific state (the state at timestep t+1)
     * @return   the transition probability
     */
    double getTransitionProbability(number x, number u, number y) const;

    /**
     * @brief    Sets probability transitions
     * @param    t_model matrices of transitions, one for each (joint) action.
     */
    void setTransitions(const std::vector<Matrix> &t_model);

    /**
     * @brief    Sets probability transitions
     * @param    t_model matrices of transitions, one for each (joint) action.
     */
    std::vector<Matrix> getTransitions();

    /**
     * @brief    Returns matrix of probability transitions for the pre-defined action.
     * @param    u A specific (joint) action
     */
    const Matrix &getTransitions(number u);

    /**
     * @brief 
     * 
     * @param x 
     * @param u 
     * @return const std::unordered_set<state>& 
     */
    const std::unordered_set<state> &getStateSuccessors(number x, number u);

  protected:
    /** @brief transition and observation matrices */
    std::vector<Matrix> t_model;

    /** @brief map from state, action pairs to set of next states */
    std::unordered_map<state, std::unordered_map<action, std::unordered_set<state>>> successor_states;
  };
} // namespace sdm
