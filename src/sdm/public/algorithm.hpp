
#pragma once

namespace sdm
{
  /**
   * @brief The public interface common to all algorithms in **SDM'Studio**.
   * 
   * Basic usage:
   * 
   * ```cpp
   * std::shared_ptr<Algorithm> algo = std::make_shared<AlgoName>(params...);
   * algo->do_initialize();
   * algo->do_solve();
   * ```
   * 
   */
  class Algorithm
  {
  public:
    virtual ~Algorithm() {}

    /**
     * @brief Initialize the algorithm.
     * 
     */
    virtual void do_initialize() = 0;

    /**
     * @brief Solve the problem.
     * 
     */
    virtual void do_solve() = 0;

    /**
     * @brief Test the result of the algorithm.
     * 
     */
    virtual void do_test() = 0;

    /**
     * @brief Save the policy in a file.
     * 
     */
    virtual void do_save() = 0;

    virtual int getTrial() = 0;

    virtual double getResult() =0;
  };
} // namespace sdm
