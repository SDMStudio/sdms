#pragma once

#include <math.h>
#include <sdm/public/algorithm.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/point_set_value_function.hpp>

namespace sdm
{
    class ValueIteration : public Algorithm
    {
    protected:
        std::shared_ptr<SolvableByHSVI> problem_;

        std::shared_ptr<sdm::PointSetValueFunction> policy_evaluation_1_;
        std::shared_ptr<sdm::PointSetValueFunction> policy_evaluation_2_;

        double error_;
        int horizon_;

        bool borne();

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

        ValueIteration(std::shared_ptr<SolvableByHSVI> problem, double error, int horizon);

        std::shared_ptr<typename sdm::PointSetValueFunction> getResult();

        double getResultOpti();

        int getTrial() { return 0; }
    };
}
#include <sdm/algorithms/value_iteration.tpp>
