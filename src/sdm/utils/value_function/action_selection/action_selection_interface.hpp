#pragma once

#include <sdm/core/function.hpp>
#include <sdm/utils/struct/pair.hpp>

namespace sdm
{
    class ValueFunctionInterface;

    class ActionSelectionInterface
    {
    public:
        
        /**
         * @brief Select the best action for a state at a precise time.
         * 
         * @param vf the value function 
         * @param state the current state
         * @param t the time step
         * @return the greedy action
         */
        virtual Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface>& vf, const std::shared_ptr<State>& state, number t) = 0;
    };
}