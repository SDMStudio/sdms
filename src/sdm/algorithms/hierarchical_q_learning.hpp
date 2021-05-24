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
#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>
#include <sdm/utils/struct/pair.hpp>

namespace sdm
{
  /**
   * @brief 
   * 
   * @tparam TState 
   * @tparam TAction 
   */
  template <typename TObservation, typename TAction>
  class HierarchicalQLearning : public Algorithm
  {
  private:
    TObservation current_obs, next_obs;
    number log_freq = 1, test_freq = 1000, save_freq = 10000;
    bool do_log = false, do_test_ = false, do_save_ = false, is_done = false;

  protected:
    /**
     * @brief The problem to be solved.
     * 
     */
    std::shared_ptr<HierarchicalPrivateOccupancyMDP<TObservation, TAction>> env_;

    /**
     * @brief Q-value function. 
     */
    std::shared_ptr<HierarchicalMappedQValueFunction<Pair<TObservation, Joint<typename TObservation::ihistory_type>>, Joint<typename TAction::output>>> q_value_table_;

    /**
     * @brief Q-value target function. 
     */
    std::shared_ptr<HierarchicalMappedQValueFunction<Pair<TObservation, Joint<typename TObservation::ihistory_type>>, Joint<typename TAction::output>>> target_q_value_table_;

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
    number planning_horizon_, step;

    double discount_, lr_, sf_, batch_size_;

    double R, E_R, q_loss;

    unsigned long global_step, max_steps_, episode, target_update_;

    std::string name_ = "qlearning";

  public:
    HierarchicalQLearning(std::shared_ptr<HierarchicalPrivateOccupancyMDP<TObservation, TAction>> &env,
              std::shared_ptr<HierarchicalMappedQValueFunction<Pair<TObservation, Joint<typename TObservation::ihistory_type>>, Joint<typename TAction::output>>> q_value,
              std::shared_ptr<HierarchicalMappedQValueFunction<Pair<TObservation, Joint<typename TObservation::ihistory_type>>, Joint<typename TAction::output>>> q_target,
              std::shared_ptr<EpsGreedy<TObservation, TAction>> exploration,
              number planning_horizon,
              double discount = 0.9,
              double lr = 0.001,
              double sf = 0.999,
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

    void do_save();


    /**
     * @brief Update the q-value functions based on the memory/experience
     * 
     */
    void update_model();

    TAction select_action(const TObservation &obs);

    Joint<HistoryTree_p<number>> get_o(JointHistoryTree_p<number> history);

    void initLogger();

    double getResultOpti() { throw sdm::exception::NotImplementedException(); }

    int getTrial() { throw sdm::exception::NotImplementedException(); }
  };
} // namespace sdm
#include <sdm/algorithms/hierarchical_q_learning.tpp>
