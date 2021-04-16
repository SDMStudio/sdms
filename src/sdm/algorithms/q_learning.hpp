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
#include <sdm/public/algorithm.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/utils/rl/exploration.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>

namespace sdm
{
  /**
   * @brief 
   * 
   * @tparam TState 
   * @tparam TAction 
   */
  template <typename TObservation, typename TAction>
  class QLearning : public Algorithm
  {
  private:
    TObservation current_obs, last_obs;
    number log_freq = 100, test_freq = 1000, save_freq = 10000;
    bool do_log = false, do_test_ = false, do_save = false, is_done = false;

  protected:
    /**
     * @brief The problem to be solved.
     * 
     */
    std::shared_ptr<GymInterface<TObservation, TAction>> env_;

    /**
     * @brief Q-value function. 
     */
    std::shared_ptr<QValueFunction<TObservation, TAction>> q_value_;

    /**
     * @brief Q-value target function. 
     */
    std::shared_ptr<QValueFunction<TObservation, TAction>> q_target_;

    /**
     * @brief Experience Memory. 
     */
    // std::shared_ptr<ReplayMemory> experience_;

    /**
     * @brief Exploration process. 
     */
    std::shared_ptr<EpsGreedy<TObservation, TAction>> exploration_process;

    /**
     * @brief Logger.
     * 
     */
    std::shared_ptr<MultiLogger> logger_;

    /**
     * @brief Some variables for the algorithm.
     * 
     */
    number planning_horizon_, step, episode;

    double discount_, lr_, batch_size_;

    unsigned long global_step, max_steps_;

    std::string name_ = "qlearning";

  public:
    QLearning(std::shared_ptr<GymInterface<TObservation, TAction>> &env,
              std::shared_ptr<QValueFunction<TObservation, TAction>> q_value,
              std::shared_ptr<QValueFunction<TObservation, TAction>> q_target,
              std::shared_ptr<EpsGreedy<TObservation, TAction>> exploration,
              number planning_horizon,
              double discount = 0.9,
              double lr = 0.001,
              double batch_size = 1,
              unsigned long num_max_steps = 10000,
              std::string name = "qlearning");

    /**
     * @brief Initialize the algorithm
     * 
     */
    void do_initialize();

    /**
     * @brief Learning procedure. Will attempt to solve the problem.
     * 
     */
    void do_solve();

    /**
     * @brief Test the result of a problem.
     * 
     */
    void do_test();

    /**
     * @brief 
     * 
     */
    void do_episode();

    /**
     * @brief 
     * 
     */
    void do_step();

    /**
     * @brief Update the q-value functions based on the memory/experience
     * 
     */
    void update_model();

    TAction select_action(const TObservation &obs);

    void initLogger();

    double getResultOpti() { throw sdm::exception::NotImplementedException(); }

    int getTrial() { throw sdm::exception::NotImplementedException(); }
  };
} // namespace sdm
#include <sdm/algorithms/q_learning.tpp>
