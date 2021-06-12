#pragma once

#include <sdm/core/function.hpp>

namespace sdm
{
    class SolvableByHSVI;
    class ValueFunction;

    class BackupInterface
    {
    public:

        /**
         * @brief Get the max value of a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::pair<double, std::shared_ptr<State>> 
         */
        virtual std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
        
        /**
         * @brief Get the best action for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return std::shared_ptr<Action> : Action
         */
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t) = 0;
    };
}