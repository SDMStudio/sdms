
#pragma once

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/world/base/mdp_interface.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    class ValueFunction;

    /**
     * @brief Public interface that must be implemented by all transformed problems that can be solved using HSVI (i.e. beliefMDP, occupancyMDP, occupancyGame, etc).
     * 
     * @tparam TState The state type
     * @tparam TAction The action type
     */
    class SolvableByDP
    {
    public:
        virtual ~SolvableByDP() {}
        
        virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const = 0;

        /**
         * @brief Get the distribution over next states
         * 
         * @param state 
         * @param action 
         * @param t 
         * @return std::shared_ptr<Distribution<std::shared_ptr<State>>> 
         */
        virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getNextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the actions availables at a specific state
         * 
         * @param state the state
         * @return the action space 
         */
        virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &state, number t) = 0;

        /**
         * @brief Get the reward at a given occupancy state and occupancy action 
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        /**
         * @brief Get the expected next value
         * 
         * @param value_function a pointer on the value function to use to perform the calculus.
         * @param state the state on which to evaluate the next expected value *
         * @param action 
         * @param t 
         * @return double 
         */
        virtual double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        /**
         * @brief Get the specific discount factor for the problem at hand
         * 
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        virtual double getDiscount(number t) = 0;
    };
} // namespace sdm