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
#include <sdm/utils/nn/dqn.hpp>
#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/rl/experience_memory_interface.hpp>


namespace sdm
{
  class DeepQLearning : public Algorithm
  {
  private:
    std::shared_ptr<Observation> observation, next_observation;
    std::shared_ptr<Action> action, next_action;
    number log_freq = 100, save_freq = 10000, target_update_freq = 10, test_n = 10;
    bool do_log_ = false, do_save_ = false, is_done = false;
    clock_t t_begin;

  protected:
    /**
     * @brief The problem to be solved.
     * 
     */
    std::shared_ptr<GymInterface> env_;

    std::shared_ptr<ExperienceMemoryInterface> experience_memory_;

    /**
     * @brief Q-value function. 
     */
    std::shared_ptr<DQN> policy_net_;

    /**
     * @brief Q-value target function. 
     */
    std::shared_ptr<DQN> target_net_;

    std::shared_ptr<QValueBackupInterface> backup_; /////

    /**
     * @brief Exploration process. 
     */
    std::shared_ptr<EpsGreedy> exploration_process;

    /**
     * @brief Logger.
     * 
     */
    std::shared_ptr<MultiLogger> logger_;

    /**
     * @brief Some variables for the algorithm.
     * 
     */
    number horizon_, step;

    double discount_, lr_, smooth_, q_value_error, value_p, R;

    std::vector<double> rewards_;

    unsigned long num_episodes_, episode;

    std::string name_ = "deepqlearning";

  public:
    DeepQLearning(std::shared_ptr<GymInterface> &env,
              std::shared_ptr<ExperienceMemoryInterface> experience_memory,
              std::shared_ptr<DQN> policy_net,
              std::shared_ptr<DQN> target_net,
              std::shared_ptr<QValueBackupInterface> backup,
              std::shared_ptr<EpsGreedy> exploration,
              number horizon,
              double discount = 0.9,
              double lr = 0.001,
              unsigned long num_episodes = 10000,
              double smooth = 0.99,
              std::string name = "deepqlearning"
            );

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

    void do_test();

    double do_evaluate();

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
     * @brief 
     * 
     */
    void do_save();

    void update_target();

    std::shared_ptr<Action> select_action(const std::shared_ptr<Observation> &observation, number t);

    void initLogger();

    double getResultOpti() { throw sdm::exception::NotImplementedException(); }

    int getTrial() { throw sdm::exception::NotImplementedException(); }

    double getResult() { throw sdm::exception::NotImplementedException(); }

  };
} // namespace sdm
