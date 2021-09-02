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
#define LOGTIME


#include <chrono>
#include <string>

#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{

  /**
   * @brief [Heuristic Search Value Iteration (HSVI)](https://arxiv.org/abs/1207.4166) and its extensions (FB-HSVI, one-sided HSVI).
   * 
   */
  class HSVI : public Algorithm,
               public std::enable_shared_from_this<HSVI>
  {
  protected:
    /** @brief The problem to be solved */
    std::shared_ptr<SolvableByHSVI> world_;

    /** @brief Lower Bound representation. */
    std::shared_ptr<ValueFunction> lower_bound_;

    /** @brief Upper Bound representation. */
    std::shared_ptr<ValueFunction> upper_bound_;

    /** @brief Some variables for the algorithm. */
    int trial, MAX_TRIALS;
    double error_, time_max_;
    number planning_horizon_, lb_update_frequency_, ub_update_frequency_;

    std::string name_ = "hsvi";
    
    bool keep_same_action_forward_backward_;

    std::shared_ptr<State> start_state;

    std::chrono::high_resolution_clock::time_point start_time, current_time;
    
    double duration;

  public:
    /**
     * @brief Construct the HSVI algorithm.
     * 
     * @param world the problem to be solved by HSVI
     * @param lower_bound the lower bound 
     * @param upper_bound the upperbound
     * @param planning_horizon the planning horizon
     * @param epsilon the error
     * @param num_max_trials the maximum number of trials before stop
     * @param name the name of the algorithm (this name is used to save logs)
     */
    HSVI(std::shared_ptr<SolvableByHSVI> &world,
         std::shared_ptr<ValueFunction> lower_bound,
         std::shared_ptr<ValueFunction> upper_bound,
         number planning_horizon,
         double epsilon,
         number num_max_trials = 10000,
         std::string name = "hsvi",
         number lb_update_frequency = 1,
         number ub_update_frequency = 1,
         double time_max = 1000, 
         bool keep_same_action_forward_backward = false);

    std::shared_ptr<HSVI> getptr();

    /**
     * @brief Initialize the algorithm. It will initialize the lower bound and the upper bound.
     */
    void do_initialize();

    /**
     * @brief Solve a problem that is solvable by HSVI. 
     */
    void do_solve();

    /**
     * @brief Test the learnt value function on one episode
     */
    void do_test();

    /**
     * @brief Save the lower bound under "name_lb.bin"
     */
    void do_save();

    /**
     * @brief Check the end of HSVI algo.
     * 
     * @param s the current state
     * @param h the current timestep
     * @return true if optimal is reached or number of trials is bigger than maximal number of trials
     * @return false elsewhere
     */
    bool do_stop(const std::shared_ptr<State> &, double /*cost_so_far*/, number);

    /**
     * @brief Explore a state.
     * 
     * @param s the state to explore
     * @param h the timestep of the exploration
     */
    void do_explore(const std::shared_ptr<State> &s, double /*cost_so_far*/, number h);

    /**
     * @brief Computes the error between bounds (or excess).
     * 
     * @param const std::shared_ptr<State> & : the state
     * @param double cost so far
     * @param number the timestep
     * @return the error
     */
    double do_excess(const std::shared_ptr<State> &, double /*cost_so_far*/, number);

    /**
     * @brief Get the lower bound value function 
     */
    std::shared_ptr<ValueFunction> getLowerBound() const;

    /**
     * @brief Get the upper bound value function 
     */
    std::shared_ptr<ValueFunction> getUpperBound() const;

    int getTrial();

    double getResult();

    void saveResults(std::string filename, double other);

    static double TIME_TO_REMOVE;
#ifdef LOGTIME

    static double TIME_IN_SELECT_STATE, TIME_IN_SELECT_ACTION, TIME_INITIALIZATION, TIME_IN_UPDATE_LB, TIME_IN_UPDATE_UB, TIME_IN_PRUNING_LB, TIME_IN_PRUNING_UB, TIME_IN_DO_EXCESS;
    static double TIME_UPDATE_BACKUP_LB, TIME_BEST_ACTION_LB, TIME_EVALUATE_LB, TIME_GET_VALUE_AT_LB, TOTAL_TIME_LB;
    static double TIME_UPDATE_BACKUP_UB, TIME_BEST_ACTION_UB, TIME_EVALUATE_UB, TIME_GET_VALUE_AT_UB, TOTAL_TIME_UB;

    /**
     * @brief Logger.
     * 
     */
    std::shared_ptr<MultiLogger> logger_, logger_precise_;

    /**
     * @brief Initialize the logger
     * 
     */
    void initLogger();

    /**
     * @brief Clean all the data Time
     * 
     */
    static void cleanTIME();

    /**
     * @brief Update the function associate with the time
     * 
     * @param start_time 
     * @param information 
     */
    void updateTime(std::chrono::high_resolution_clock::time_point start_time, std::string information);
#endif

  };
} // namespace sdm
