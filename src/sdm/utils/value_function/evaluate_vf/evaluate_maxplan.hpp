#pragma once

#include <sdm/utils/value_function/evaluate_vf/evaluate_vf_interface.hpp>

namespace sdm
{
    class EvaluateMaxplanInterface : public EvaluateVFInterface
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
        Pair<std::shared_ptr<State>,double> evaluate(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);
    };
}