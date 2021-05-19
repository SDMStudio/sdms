
#pragma once

namespace sdm
{
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

    // /**
    //  * @brief Get the optimal resultat of the algorithm, but only after the resolution of the problem (do_solve()).
    //  *
    //  * @return double
    //  */
    // virtual double getResultOpti() =0;

    virtual int getTrial() = 0;
  };
} // namespace sdm
