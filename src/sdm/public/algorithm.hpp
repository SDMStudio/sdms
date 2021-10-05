
#pragma once

#include <string>
#include <chrono>
#include <iostream>

namespace sdm
{
  /**
   * @brief The public interface common to all algorithms in **SDM'Studio**.
   * 
   * Basic usage:
   * 
   * ```cpp
   * std::shared_ptr<Algorithm> algo = std::make_shared<AlgoName>(params...);
   * algo->initialize();
   * algo->solve();
   * ```
   * 
   */
  class Algorithm
  {
  public:

    Algorithm(std::string name);
  
    virtual ~Algorithm();

    /**
     * @brief Initialize the algorithm. 
     * 
     * Initialize the algorithm. This method must be called before `solve()`.
     * In fact, this will initialize components required to run the algorithm. 
     * 
     */
    virtual void initialize() = 0;

    /**
     * @brief Learning procedure. Will attempt to solve the problem.
     * 
     * This method will solve a problem using the current algorithm. Before being 
     * able to call `solve()`, the algorithm must be initialize with the function 
     * `initialize()`.
     * 
     */
    virtual void solve() = 0;

    /**
     * @brief Test the current policy and display the reward obtained.
     */
    virtual void test() = 0;

    /**
     * @brief Save the value function.
     * 
     */
    virtual void save() = 0;

    /**
     * @brief Get the name of the algorithm.
     * 
     * This function will return the name provided by the user at 
     * the instanciation. This name is used by the logger to print
     * logs in a file with this name. It is also the name of the file
     * containing the model and the configuration of the algorithm.
     * 
     * @return the name of the algorithm 
     */
    std::string getName() const;

    void startExecutionTime();
    void stopExecutionTime();

    std::chrono::high_resolution_clock::time_point getStartExectionTime() const;
    std::chrono::high_resolution_clock::time_point getStopExectionTime() const;
    
    double getExecutionTime() const;

  protected:
    std::string name = "hsvi";
    
    std::chrono::high_resolution_clock::time_point start_execution_time, stop_execution_time;
            
  };
} // namespace sdm
