
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/world/world_type.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/exception.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{

    /**
     * @brief Public interface that must be implemented by all transformed problems that can be solved using HSVI (i.e. beliefMDP, occupancyMDP, occupancyGame, etc).
     * 
     * @tparam TState The state type
     * @tparam TAction The action type
     */
    template <typename TState, typename TAction>
    class SolvableByHSVI
    {
    public:
        virtual ~SolvableByHSVI() {}
        
        /**
         * @brief Get the initial state
         */
        virtual TState getInitialState() = 0;

        /**
         * @brief Get the next occupancy state.
         * 
         * @param state the occupancy state
         * @param action the action state
         * @param t the timestep
         * @param hsvi a pointer on the algorithm that makes the call
         * @return the next occupancy state
         */
        virtual TState nextState(const TState &state, const TAction &action, int t = 0, HSVI<TState, TAction> *hsvi = nullptr) const = 0;

        /**
         * @brief Get the actions availables at a specific state
         * 
         * @param state the state
         * @return the action space 
         */
        virtual std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &state) = 0;

        /**
         * @brief Get the reward at a given occupancy state and occupancy action 
         */
        virtual double getReward(const TState &state, const TAction &action) const = 0;

        /**
         * @brief Get the expected next value
         * 
         * @param value_function a pointer on the value function to use to perform the calculus.
         * @param state the state on which to evaluate the next expected value *
         * @param action 
         * @param t 
         * @return double 
         */
        virtual double getExpectedNextValue(ValueFunction<TState, TAction> *value_function, const TState &state, const TAction &action, int t = 0) const = 0;

        /**
         * @brief Get the underlying problem. For instance the underlying DecPOMDP of the OccupancyMDP or the underlying POMDP of the current BeliefMDP.  
         * 
         * @return the underlying problem 
         */
        virtual typename WorldType<TState, TAction>::underlying_problem_type *getUnderlyingProblem() = 0;

        /**
         * @brief Check if the problem is serialized.
         * 
         * @return true if the problem is serialized.
         * @return false if the problem is not serialized.
         */
        virtual bool isSerialized() const = 0;
    };
} // namespace sdm