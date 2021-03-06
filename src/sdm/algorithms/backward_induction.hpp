#pragma once

#include <sdm/types.hpp>
#include <sdm/algorithms/planning/dp.hpp>
#include <sdm/utils/logging/logger.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>

namespace sdm
{

  /**
   * @brief The algorithm [Backward Induction](https://en.wikipedia.org/wiki/Backward_induction). 
   */
  class BackwardInduction : public DynamicProgramming,
                            public std::enable_shared_from_this<BackwardInduction>
  {
  public:
    /**
     * @brief Construct the Backward Induction algorithm.
     * 
     * @param world the problem to be solved by Backward induction
     * @param name the name of the algorithm (this name is used to save logs)
     */
    BackwardInduction(const std::shared_ptr<SolvableByHSVI> &world,
                      std::string name = "backward induction");

    std::shared_ptr<BackwardInduction> getptr();

    /**
     * @brief Initialize the algorithm.
     * 
     */
    void initialize();

    /**
     * @brief Solve a problem with backward induction algorithm. 
     */
    void solve();

    /**
     * @brief Test the learnt value function on one episode
     */
    void test();

    /**
     * @brief Save the lower bound under "name_lb.bin"
     * 
     */
    void save();

    /**
     * @brief Check the end of HSVI algo
     * 
     * @param s the current state
     * @param h the current timestep
     * @return true if optimal is reached or number of trials is bigger than maximal number of trials
     * @return false elsewhere
     */
    bool stop(const std::shared_ptr<State> &, double /*cost_so_far*/, number);

    /**
     * @brief Explore a state.
     * 
     * @param s the state to explore
     * @param h the timestep of the exploration
     */
    void explore(const std::shared_ptr<State> &s, double /*cost_so_far*/, number h);

    // double backward_induction(const std::shared_ptr<State> &s, number h);

    /**
     * @brief Get the bound value function 
     */
    std::shared_ptr<ValueFunction> getBound() const;

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

  protected:
    /**
     * @brief Initialize the logger
     */
    void initLogger();

    number LOG_DEPTH = 2;

    /**
     * @brief representation. 
     */
    std::shared_ptr<TabularValueFunction> bound_;

    /**
     * @brief Some variables for the algorithm.
     * 
     */
    std::string name_ = "backward_induction";

    std::shared_ptr<State> start_state;
  };
} // namespace sdm
