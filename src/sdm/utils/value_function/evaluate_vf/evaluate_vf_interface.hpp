#pragma once

#include <sdm/core/function.hpp>

namespace sdm
{
    class ValueFunction;

    class EvaluateVFInterface
    {
    public:

        /**
         * @brief Evaluation of a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::pair<double, std::shared_ptr<State>> 
         */
        virtual Pair<std::shared_ptr<State>,double> evaluate(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
    };
}