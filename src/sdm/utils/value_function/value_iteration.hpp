#pragma once

#include <math.h>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/initializers.hpp>


namespace sdm
{
    template <typename TState, typename TAction>
    class ValueIteration
    {
        protected :

        std::shared_ptr<SolvableByHSVI<TState, TAction>> problem_;

        //void policy_iteration_inf();

        std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>> policy_iteration_non_inf();

        public :

            ValueIteration(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem,double discount, double error, int horizon);

            std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>>policy_iteration();

            //bool borne(MappedVector<TState,double> policy_evaluation_tempo,MappedVector<TState,double> policy_evaluation,double error = 0.1);

            bool borne(std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>> policy_evaluation_tempo,std::shared_ptr<typename sdm::MappedValueFunction<TState, TAction>> policy_evaluation,double error = 0.1);

    };
}
#include <sdm/utils/value_function/value_iteration.tpp>