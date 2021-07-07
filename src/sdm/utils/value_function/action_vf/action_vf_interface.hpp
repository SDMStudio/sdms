#pragma once

#include <sdm/core/function.hpp>

namespace sdm
{
    class ValueFunction;

    class ActionVFInterface
    {
    public:
        
        /**
         * @brief Select the best action for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::shared_ptr<Action> : Action
         */
        virtual std::shared_ptr<Action> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
    };
}