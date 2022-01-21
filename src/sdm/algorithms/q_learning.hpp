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
#include <sdm/utils/rl/experience_memory_interface.hpp>

namespace sdm
{
  /**
   * @brief Q-Learning and its extensions (DQN, etc).
   *
   * This class contains the general algorithmic scheme of Q-learning. By redefining
   * the Q-value function or the environment, we will be able to obtain different
   * state-of-the-art algorithms such as: Q-learning, DQN.
   *
   */
  class QLearning : public Algorithm
  {
  public:
    QLearning(const std::shared_ptr<GymInterface> &env,
              std::shared_ptr<ExperienceMemoryInterface> experience_memory,
              std::shared_ptr<QValueFunction> q_value,
              std::shared_ptr<QValueFunction> q_target,
              std::shared_ptr<EpsGreedy> exploration,
              number horizon,
              double rate_start = 1.0,
              double rate_end = 0.001,
              double rate_decay = 1000,
              unsigned long num_episodes = 10000,
              std::string name = "qlearning");

    /**
     * @brief Initialize the algorithm.
     *
     * Initialize the algorithm. This method must be called before `solve()`.
     * In fact, this will initialize value functions and other components required
     * by the algorithm.
     *
     */
    virtual void initialize();

    /**
     * @brief Learning procedure. Will attempt to solve the problem.
     *
     * This method will solve a problem using q-learning. Before calling `solve()`,
     * the algorithm must be initialize with `initialize()` function.
     *
     */
    virtual void solve();

    /**
     * @brief Execute an episode.
     *
     * An episode consists of several stages. The length of the episode depends on the
     * problem. In most problems, it is a constant equal to a value called the planning
     * horizon. But, in few cases, it can vary according to actions taken by the agents.
     *
     */
    virtual void doEpisode();
    virtual void doEpisodeRecursive(const std::shared_ptr<State> &observation, number t);


    /**
     * @brief Finalize the episode
     *
     * This function will increment the number of episodes executed and update log variables.
     *
     */
    virtual void endEpisode();

    /**
     * @brief Execute a learning step.
     *
     * One step consists of several subroutines. First, select the action according
     * to the observation and policy. Then, execute the action and get next observation
     * and reward. Finally, add the sequence perceived in the replay memory and update
     * the value functions.
     *
     */
    virtual void doStep();

    /**
     * @brief Finalize the step.
     *
     * This function will increment the number of steps executed and update log variables.
     *
     */
    virtual void endStep();

    /**
     * @brief Test the current policy and display the reward obtained.
     */
    virtual void test();

    /**
     * @brief Save the value function.
     *
     */
    virtual void save();

    /**
     * @brief Update the target model.
     *
     * Copy the content of the q-value function into the target q-value function.
     *
     */
    virtual void updateTarget();

    /**
     * @brief Select an action according to the current policy and the exploration process.
     *
     * The espilon greedy exploration is often used in Q-learning. However, other exploration
     * processes can be equally relevant for solving some particular problems. One example is
     * the use of an expert strategy as an more sophisticated selection method.
     *
     * @param observation the current observation
     * @param t the timestep
     * @return the selected action
     */
    virtual std::shared_ptr<Action> selectAction(const std::shared_ptr<State> &state, number t);

    /**
     * @brief Initialize the loggers.
     *
     * The loggers are objects allowing to write the statistics relative to the execution of the
     * algorithm in files. The files in question can have different formats (text, XML, CSV, etc.)
     * depending on the desired processing afterwards.
     *
     */
    virtual void initLogger();

    /**
     * @brief Get the environment
     *
     * @return the gym environment
     */
    std::shared_ptr<GymInterface> getEnv() const;

    /**
     * @brief Get the name of the algorithm as a string.
     *
     * This function will return the name of the algorithm as a string.
     * It does not return the name of a specific instance (`name` attribute)
     * but those of the general algorithm used (i.e. HSVI, QLearning, etc).
     *
     * @return the algorithm name
     */
    std::string getAlgorithmName();

    virtual void logging();

    virtual void printStartInfo();

    virtual void printEndInfo();

  protected:
    /** @brief The problem to be solved */
    std::shared_ptr<GymInterface> env_;

    /** @brief The experience memory */
    std::shared_ptr<ExperienceMemoryInterface> experience_memory_;

    /** @brief Q-value function. */
    std::shared_ptr<QValueFunction> q_value_, q_target_;

    /** @brief The exploration process. */
    std::shared_ptr<EpsGreedy> exploration_process;

    /** @brief The logger */
    std::shared_ptr<MultiLogger> logger_;

    /**
     * @brief Some hyperparameters for the algorithm.
     */
    number horizon_, step;

    unsigned long global_step, num_episodes_, episode;

    double learning_rate, rate_start, rate_end, rate_decay;

    std::shared_ptr<State> observation;

    number log_freq = 10, test_freq = 10000, save_freq = 10000, max_num_steps_by_ep_ = 200, target_update_freq = 1;

    bool do_log_ = false, do_test_ = false, do_save_ = false, is_done = false;

    clock_t t_begin;
  };
} // namespace sdm
