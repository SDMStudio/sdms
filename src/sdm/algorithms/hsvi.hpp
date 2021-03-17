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
#include <sdm/public/algorithm.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/utils/value_function/value_function.hpp>

namespace sdm
{

  template <typename TState, typename TAction>
  class SolvableByHSVI;

  /**
   * @brief 
   * 
   * @tparam TState 
   * @tparam TAction 
   */
  template <typename TState, typename TAction>
  class HSVI : public Algorithm
  {
  protected:
    /**
     * @brief The problem to be solved.
     * 
     */
    std::shared_ptr<SolvableByHSVI<TState, TAction>> world_;

    /**
     * @brief Lower Bound representation. 
     */
    std::shared_ptr<ValueFunction<TState, TAction>> lower_bound_;

    /**
     * @brief Upper Bound representation. 
     */
    std::shared_ptr<ValueFunction<TState, TAction>> upper_bound_;

    /**
     * @brief Some variables for the algorithm.
     * 
     */
    int trial, MAX_TRIALS;
    double error_;
    number planning_horizon_;

    void initLogger();

  public:
    /**
     * @brief Construct a new HSVI object
     * 
     * @param trials 
     * @param results 
     */
    // HSVI(number trials, std::string results);

    /**
     * @brief Construct a new HSVI object
     * 
     * @param world the problem we want to solve
     * @param lb 
     * @param ub 
     * @param trials 
     * @param results 
     */
    HSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> &world,
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
    double do_excess(const TState &s, number h);

    /**
     * @brief Check the end of HSVI algo
     * 
     * @param h 
     * @return true if optimal is reached or number of trials is bigger than maximal number of trials
     * @return false 
     */
    bool do_stop(const TState &s, number h);

    /**
     * @brief Explore state
     * 
     * @param s 
     * @param h 
     */
    void do_explore(const TState &s, number h);

    void do_test();

    /**
     * @brief Select the greedy action
     * 
     * @param s 
     * @return TAction 
     */
    TAction selectNextAction(const TState &s, number h);

    /**
     * @brief Select the next state to explore 
     * 
     * @param s 
     * @param a 
     * @return TState 
     */
    TState selectNextState(const TState &s, const TAction &
    a, number d);
  };
} // namespace sdm
#include <sdm/algorithms/hsvi.tpp>
