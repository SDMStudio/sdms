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

#include <string>

#include <sdm/types.hpp>
#include <sdm/world/posg.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/core/state/state.hpp>

namespace sdm
{
  /**
   * @brief 
   * 
   * @tparam TState 
   * @tparam TAction 
   */
  template <typename TState, typename TAction>
  class HSVI
  {
  protected:
    /**
     * @brief Lower Bound representation. 
     */
    std::shared_ptr<ValueFunction<TState, TAction>> lower_bound_;

    /**
     * @brief Upper Bound representation. 
     */
    std::shared_ptr<ValueFunction<TState, TAction>> upper_bound_;

    /**
     * @brief Heuristic search for HSVI.
     * 
     */
    // std::shared_ptr<HeuristicSearch<TState, TAction>> search;

    /**
     * @brief The problem to be solved.
     * 
     */
    std::shared_ptr<POSG> world_;

    /**
     * @brief Some variables for the algorithm.
     * 
     */
    int trial, MAX_TRIALS;
    double error_;
    number planning_horizon_;

    void initLogger();
    void updateValueFunction(TState s, number h);

  public:
    /**
     * @brief Construct a new HSVI object
     * 
     * @param trials 
     * @param results 
     */
    HSVI(number trials, std::string results);

    /**
     * @brief Construct a new HSVI object
     * 
     * @param world the problem we want to solve
     * @param lb 
     * @param ub 
     * @param trials 
     * @param results 
     */
    HSVI(std::shared_ptr<POSG> &world,
         std::shared_ptr<ValueFunction<TState, TAction>> lower_bound,
         std::shared_ptr<ValueFunction<TState, TAction>> upper_bound,
         number planning_horizon,
         double epsilon,
         number num_max_trials = 10000);

    /**
     * @brief Initialize the algorithm
     * 
     * @param tr_problem 
     */
    void do_initialize();

    /**
     * @brief Solve the transformed problem
     * 
     * @param tr_problem 
     * @param planning_horizon 
     * @param epsilon 
     * @param discount 
     */
    void do_solve();

    /**
     * @brief 
     * 
     * @param s 
     * @return double 
     */
    double do_excess(TState s, number h);

    /**
     * @brief Check the end of HSVI algo
     * 
     * @param h 
     * @return true if optimal is reached or number of trials is bigger than maximal number of trials
     * @return false 
     */
    bool do_stop(TState s, number h);

    /**
     * @brief Explore state
     * 
     * @param s 
     * @param h 
     */
    void do_explore(TState s, number h);

    /**
     * @brief Select the greedy action
     * 
     * @param s 
     * @return TAction 
     */
    TAction selectNextAction(TState s, number h);

    /**
     * @brief Select the next state to explore 
     * 
     * @param s 
     * @param a 
     * @return TState 
     */
    TState selectNextState(TState s, TAction a, number d);

    TState getInitialState();
  };

  // using mdpHSVI = HSVI<number, number>;
  // using HSVI = HSVI<BeliefState, number>;
  // using oHSVI = HSVI<oState, DetDecisionRule>;
} // namespace sdm
