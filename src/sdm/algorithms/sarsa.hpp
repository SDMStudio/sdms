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
#include <sdm/algorithms/q_learning.hpp>
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
  class SARSA : public QLearning
  {
  public:
    SARSA(const std::shared_ptr<GymInterface> &env,
          std::shared_ptr<ExperienceMemoryInterface> experience_memory,
          std::shared_ptr<QValueFunction> q_value,
          std::shared_ptr<QValueFunction> q_target,
          std::shared_ptr<EpsGreedy> exploration,
          number horizon,
          double rate_start = 1.0,
          double rate_end = 0.001,
          double rate_decay = 1000,
          unsigned long num_episodes = 10000,
          std::string name = "sarsa");

    /**
     * @brief Initialize the algorithm.
     *
     * Initialize the algorithm. This method must be called before `solve()`.
     * In fact, this will initialize value functions and other components required
     * by the algorithm.
     *
     */
    void initialize();

    /**
     * @brief Execute an episode.
     *
     * An episode consists of several stages. The length of the episode depends on the
     * problem. In most problems, it is a constant equal to a value called the planning
     * horizon. But, in few cases, it can vary according to actions taken by the agents.
     *
     */
    void doEpisodeRecursive(const std::shared_ptr<State> &observation, number t);

    /**
     * @brief Finalize the episode
     *
     * This function will increment the number of episodes executed and update log variables.
     *
     */
    void endEpisode();

    std::string getAlgorithmName();

    void initLogger();
    void logging();

  protected:
    std::vector<std::shared_ptr<Action>> actors;

    double cumul_reward, current;

    std::shared_ptr<Action> default_decision_rule(number t) const;
  };
} // namespace sdm
