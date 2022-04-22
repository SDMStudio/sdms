
#pragma once

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/distribution.hpp>
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
     * @brief Public interface that must be implemented by all transformed problems that can be solved 
     * using HSVI (i.e. beliefMDP, occupancyMDP, occupancyGame, etc).
     * 
     * @tparam TState The state type
     * @tparam TAction The action type
     */
    class SolvableByDP : public GymInterface, public std::enable_shared_from_this<SolvableByDP>
    {
    public:
        virtual ~SolvableByDP() {}

        /**
         * @brief Get the horizon.
         */
        virtual number getHorizon() const = 0;

        /**
         * @brief Get the specific discount factor for the problem at hand
         * 
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        virtual double getDiscount(number t) const = 0;

        /**
         * @brief Get the initial state.
         */
        virtual std::shared_ptr<State> getInitialState() = 0;

        /** @brief Set the initial state */
        virtual void setInitialState(const std::shared_ptr<State> &) = 0;

        /** @brief Get the start distribution */
        virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const = 0;

        /**
         * @brief Get the action space at a specific state and timestep.
         * 
         * The state dependency is required when the game forbid the usage of a number of actions in 
         * this state. It is also used in some reformulated problems where actions are decision rules.
         * The time dependency is required in extensive-form games in which some agents have a different 
         * action space.   
         * 
         * @param state the state
         * @param t the time step
         * @return the action space 
         */
        virtual std::shared_ptr<ActionSpace> getActionSpaceAt(const std::shared_ptr<State> &state, number t) = 0;

        /**
         * @brief Get the observation space at a specific state, action and time step.
         * 
         * @param state the state
         * @param action the action
         * @param t the time step
         * @return the observation space
         */
        virtual std::shared_ptr<ObservationSpace> getObservationSpaceAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) = 0;

        /**
         * @brief Get the distribution over next states
         * 
         * @param state  the state
         * @param action  the action
         * @param t the time step
         * @return std::shared_ptr<Distribution<std::shared_ptr<State>>> 
         */
        // virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getNextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const = 0;

        /**
         * @brief Get the next state and corresponding probability.
         * 
         * Return the next state s'= T(s,a,o) and p(o | s, a)=p(s'| s, a) (dans ceratins cas mais pas tous)
         * 
         * @param state the state
         * @param action the action
         * @param observation the observation
         * @param t the time step
         * @return the pair of state and probability 
         */
        virtual Pair<std::shared_ptr<State>, double> getNextStateAndProba(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t) = 0;

        /**
         * @brief Get the well defined underlying problem. 
         * 
         * Some problems are solvable by DP algorithms even if they are not well defined. Usually, they 
         * simply are reformulation of an underlying well defined problem. For instance, the underlying 
         * DecPOMDP of the OccupancyMDP or the underlying POMDP of the current BeliefMDP.  
         * 
         * @return the underlying problem 
         */
        virtual const std::shared_ptr<MDPInterface> &getUnderlyingProblem() const = 0;

        /** ---------- FOR BELLMAN OPERATOR -------------- */

        /**
         * @brief Get the reward of executing a specific action in an specific state at timestep t. 
         * 
         * The time dependency can be required in non-stationnary problems.   
         * 
         * @param state the state
         * @param action the action
         * @param t the timestep
         * @return the reward
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) = 0;

        /**
         * @brief Get the expected next value
         * 
         * @param value_function a pointer on the value function to use to perform the calculus.
         * @param state the state on which to evaluate the next expected value
         * @param action the action
         * @param t the time step
         * @return the expected next value
         */
        virtual double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) = 0;

        /**
         * @brief Checks if the problem has a finite horizon.
         * 
         * The problem has a finite horizon if the horizon is set to a value 
         * greater than zero.
         * 
         */
        inline bool isFiniteHorizon() const { return (this->getHorizon() > 0); }

        /**
         * @brief Checks if the problem has an infinite horizon.
         * 
         * The problem has an infinite horizon if the horizon is set to zero.
         * 
         */
        inline bool isInfiniteHorizon() const { return !isFiniteHorizon(); }
    };
} // namespace sdm