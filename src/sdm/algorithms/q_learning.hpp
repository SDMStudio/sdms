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

#include <sdm/types.hpp>
#include <sdm/world/gym_interface.hpp>

namespace sdm
{
  /**
   * @brief 
   * 
   * @tparam TState 
   * @tparam TAction 
   */
  template <typename TState, typename TAction, typename TObservation>
  class QLearning : public LearningAlgo<TState, TAction, TObservation>
  {
  protected:
    /**
     * @brief Lower Bound representation. 
     */
    std::shared_ptr<QValueFunction<TState, TAction>> q_value_, q_target_;

  public:
    /**
     * @brief Construct a new QLearning object
     * 
     * @param trials 
     * @param results 
     */
    QLearning(number trials, std::string results);

    /**
     * @brief Construct a new QLearning object
     * 
     * @param world the problem we want to solve
     * @param lb 
     * @param ub 
     * @param trials 
     * @param results 
     */
    QLearning(std::shared_ptr<GymInterface> &env,
              std::shared_ptr<QValueFunction<TState, TAction>> value,
              std::shared_ptr<QValueFunction<TState, TAction>> target,
              number planning_horizon,
              double epsilon,
              number num_max_trials = 10000);
  };
} // namespace sdm
#include <sdm/algorithms/q_learning.tpp>
