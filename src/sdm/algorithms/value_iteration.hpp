#pragma once

#include <math.h>
#include <sdm/public/algorithm.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    /**
     * @brief [Value Iteration](https://www.jstor.org/stable/24900506) for MDP.
     */
    class ValueIteration : public Algorithm
    {
    protected:
        /** @brief The problem to be solved */
        std::shared_ptr<SolvableByHSVI> problem_;

        /** @brief The value function */
        std::shared_ptr<sdm::TabularValueFunction> policy_evaluation_1_;

        /** @brief The copy of the value function */
        std::shared_ptr<sdm::TabularValueFunction> policy_evaluation_2_;

        double error_;
        
        int horizon_;

        bool borne();

        std::vector<std::vector<std::shared_ptr<State>>> all_state;

    public:
        /**
         * @brief Initialize the algorithm
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
         * @brief Save the value function. 
         */
        void do_save() {}

        void determinedAllNextState();
        void determinedAllNextStateRecursive(const std::shared_ptr<State> &, number t);

        ValueIteration(std::shared_ptr<SolvableByHSVI> problem, double error, int horizon);

        std::shared_ptr<ValueFunction> getValueFunction();

        double getResult();

        int getTrial() { return 0; }
    };
}