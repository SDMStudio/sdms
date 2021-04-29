#pragma once

#include <math.h>
#include <sdm/public/algorithm.hpp>

#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/initializers.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>


namespace sdm
{
    template <typename TState, typename TAction>
    class ValueIteration : public Algorithm
    {
        protected :

        std::shared_ptr<SolvableByHSVI<TState, TAction>> problem_;

        std::shared_ptr<sdm::MappedValueFunction<TState, TAction>> policy_evaluation_1_;
        std::shared_ptr<sdm::MappedValueFunction<TState, TAction>> policy_evaluation_2_;

        double error_;
        int horizon_;

        bool borne();

        public :

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

            ValueIteration(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem,double discount, double error, int horizon);

            std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>> getResult();

            double getResultOpti();

            int getTrial() {return 0;}
    };
}
#include <sdm/algorithms/value_iteration.tpp>
