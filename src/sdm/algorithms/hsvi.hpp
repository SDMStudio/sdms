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
#include <sdm/utils/logging/logger.hpp>

namespace sdm
{

  /**
   * @brief 
   * 
   * @tparam TState 
   * @tparam TAction 
   */
  template <typename TState, typename TAction>
  class HSVI : public Algorithm,
               public std::enable_shared_from_this<HSVI<TState, TAction>>
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
     * @brief Logger.
     * 
     */
    std::shared_ptr<MultiLogger> logger_;

    /**
     * @brief Some variables for the algorithm.
     * 
     */
    int trial, MAX_TRIALS;
    double error_;
    number planning_horizon_;
    std::string name_ = "hsvi";

    void initLogger();

  public:
    /**
     * @brief Construct the HSVI object.
     * 
     * @param world the problem to be solved by HSVI
     * @param lower_bound the lower bound 
     * @param upper_bound the upperbound
     * @param planning_horizon the planning horizon
     * @param epsilon the error
     * @param num_max_trials the maximum number of trials before stop
     * @param name the name of the algorithm (this name is used to save logs)
     */
    HSVI(std::shared_ptr<SolvableByHSVI<TState, TAction>> &world,
         std::shared_ptr<ValueFunction<TState, TAction>> lower_bound,
         std::shared_ptr<ValueFunction<TState, TAction>> upper_bound,
         number planning_horizon,
         double epsilon,
         number num_max_trials = 10000,
         std::string name = "hsvi");

    std::shared_ptr<HSVI<TState, TAction>> getptr();

    /**
     * @brief 
     * 
     */
    void do_initialize();

    /**
     * @brief Solve a problem solvable by HSVI. 
     */
    void do_solve();

    /**
     * @brief Test the learnt value function on one episode
     */
    void do_test();

    /**
     * @brief Computes the error between bounds (or excess).
     * 
     * @param s the state
     * @param t the timestep
     * @return the error
     */
    double do_excess(const TState &, number);

    /**
     * @brief Check the end of HSVI algo
     * 
     * @param s the current state
     * @param h the current timestep
     * @return true if optimal is reached or number of trials is bigger than maximal number of trials
     * @return false elsewhere
     */
    bool do_stop(const TState &, number);

    /**
     * @brief Explore a state.
     * 
     * @param s the state to explore
     * @param h the timestep of the exploration
     */
    void do_explore(const TState &s, number h);

    //Pour le moment, je supprime pas les autres pour tester avec le gt
    void do_explore(const TState &, number, double);
    bool do_stop(const TState &, number, double);
    double do_excess_2(const TState &, number , double);
    /**
     * @brief Select the next action
     * 
     * @param s the current state
     * @param h the current timestep
     * @return TAction 
     */
    TAction selectNextAction(const TState &, number );

    /**
     * @brief Select the next state to explore 
     * 
     * @param s the current state
     * @param a the current action
     * @param h the current timestep
     * @return the next state
     */
    TState selectNextState(const TState &, const TAction &, number);

    /**
     * @brief Get the lower bound value function 
     */
    std::shared_ptr<ValueFunction<TState, TAction>> getLowerBound() const;

    /**
     * @brief Get the upper bound value function 
     */
    std::shared_ptr<ValueFunction<TState, TAction>> getUpperBound() const;

    int getTrial();

    double getResultOpti();
  };
} // namespace sdm
#include <sdm/algorithms/hsvi.tpp>
