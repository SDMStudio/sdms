
#pragma once

namespace sdm
{
  class Algorithm
  {
  public:
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
  };
} // namespace sdm
