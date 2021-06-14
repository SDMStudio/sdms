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
#include <sdm/core/dynamics/state_dynamics_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/core/distribution.hpp>

namespace sdm
{
  /**
   * @brief This class provide quick accessors to transition probability distributions.
   * 
   */
  class TabularStateDynamics : public StateDynamicsInterface
  {
  public:
    using value_type = double;
    using matrix_type = MappedMatrix<std::shared_ptr<State>, std::shared_ptr<State>, value_type>;

    TabularStateDynamics();

    TabularStateDynamics(const TabularStateDynamics &copy);

    virtual ~TabularStateDynamics();

    /**
     * @brief    Set the transition probability on one point.
     * @param    state A specific state (the state at timestep t)
     * @param    action A specific (joint) action
     * @param    next_state A specific state (the state at timestep t+1)
     * @param    proba probability of the transition
     * @param    cumul whether or not we cumulate probabilities.
     */
    void setTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, double proba, number t = 0, bool cumul = false);

    /**
     * @brief    Get the transition probability on one point.
     * @param    state A specific state (the state at timestep t)
     * @param    action A specific (joint) action
     * @param    next_state A specific state (the state at timestep t+1)
     * @return   the transition probability
     */
    double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;

    /**
     * @brief Get the list of all reachable states 
     * 
     * @param state the current state
     * @param action the current action
     * @return the list of accessible states 
     */
    std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

    /**
     * @brief Create the Reachable State, i.e. a state and an action will be associated with a next_state
     * 
     * @param  state A specific state (the state at timestep t)
     * @param  action A specific action
     * @param  next_state A specific state (the state at timestep t+1)
     * @param t Timestep t 
     */
    void setReachablesStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action,const std::shared_ptr<State> &next_state, number t = 0);

    std::shared_ptr<Distribution<std::shared_ptr<State>>> getNextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action);

  protected:
    /** @brief transition and observation matrices */
    std::unordered_map<std::shared_ptr<Action>, matrix_type> t_model;

    /** @brief map from state, action pairs to set of next states */
    std::unordered_map<std::shared_ptr<State>, std::unordered_map<std::shared_ptr<Action>, std::set<std::shared_ptr<State>>>> successor_states;
  };

} // namespace sdm
