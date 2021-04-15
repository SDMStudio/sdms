#pragma once

#include <math.h>
#include <sdm/types.hpp>
#include <sdm/public/algorithm.hpp>

#include <sdm/utils/value_function/tabular_value_function.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    class ValueIteration : public Algorithm
    {
    protected:
        std::shared_ptr<SolvableByHSVI<TState, TAction>> problem_;

        std::shared_ptr<sdm::MappedValueFunction<TState, TAction>> policy_evaluation_1_;
        std::shared_ptr<sdm::MappedValueFunction<TState, TAction>> policy_evaluation_2_;

        double error_;

        bool borne();

        //void policy_iteration_inf();

        //std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>> policy_iteration_non_inf();

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

        ValueIteration(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, double discount, double error, number horizon);

        //std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>>policy_iteration();

        std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>> getResult();

        double getResultOpti();

        int getTrial() { return 0; }
    };
}
#include <sdm/algorithms/value_iteration.tpp>