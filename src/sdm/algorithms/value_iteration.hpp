#pragma once

#include <math.h>
#include <sdm/public/algorithm.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    class ValueIteration : public Algorithm
    {
    protected:
        std::shared_ptr<SolvableByHSVI> problem_;

        std::shared_ptr<sdm::TabularValueFunction> policy_evaluation_1_;
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

        void do_save() {}

        void determinedAllNextState();
        void determinedAllNextStateRecursive(const std::shared_ptr<State>& , number t);

        ValueIteration(std::shared_ptr<SolvableByHSVI> problem, double error, int horizon);

        std::shared_ptr<ValueFunction> getValueFunction();

        double getResult();

        int getTrial() { return 0; }
    };
}