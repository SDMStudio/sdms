#pragma once

#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{

  /**
   * @brief 
   * 
   * @tparam std::shared_ptr<State> 
   * @tparam std::shared_ptr<Action> 
   */
  class BackwardInduction : public Algorithm, public std::enable_shared_from_this<BackwardInduction>
  {
  protected:
    /**
     * @brief The problem to be solved.
     * 
     */
    std::shared_ptr<SolvableByHSVI> world_;

    /**
     * @brief representation. 
     */
    std::shared_ptr<TabularValueFunction> bound_;

    /**
     * @brief Logger.
     * 
     */
    std::shared_ptr<MultiLogger> logger_, logger_precise_;

    /**
     * @brief Some variables for the algorithm.
     * 
     */
    number planning_horizon_;
    std::string name_ = "backward_induction";

    std::shared_ptr<State> start_state;

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
    BackwardInduction(std::shared_ptr<SolvableByHSVI> &world,
         number planning_horizon,
         std::string name = "backward induction");

    std::shared_ptr<BackwardInduction> getptr();

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
     * @brief Save the lower bound under "name_lb.bin"
     * 
     */
    void do_save();

    /**
     * @brief Check the end of HSVI algo
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
    double backward_induction(const std::shared_ptr<State> &s, number h);

    /**
     * @brief Computes the error between bounds (or excess).
     * 
     * @param const std::shared_ptr<State> & : the state
     * @param double : cost so far
     * @param number : the timestep
     * @return the error
     */
    double do_excess(const std::shared_ptr<State> &, double /*cost_so_far*/, number);

    /**
     * @brief Get the bound value function 
     */
    std::shared_ptr<ValueFunction> getBound() const;

    int getTrial(){return 0;}

    double getResult();

    void saveResults(std::string filename, double other);
    
  };
} // namespace sdm
