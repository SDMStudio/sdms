#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/distribution.hpp>

namespace sdm
{
    /**
     * @brief This class provides a common interface for every models of state dynamics.
     * A model of state transition is well defined if it implements this interface. The state dynamics is required to define a MDP (see file MDP.hpp) and related problems.
     * Usually state transitions depend on the time, the previous state and the action. However, in few situations the transition can only depend on one or two of these parameters.
     */
    class StateDynamicsInterface
    {
    public:
        /**
         * @brief Get reachable states from a state given a specific action. 
         * 
         * @param state the current state
         * @param action the current action
         * @param t the timestep
         * @return the list of next reachable states 
         */
        virtual std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        /**
         * @brief Get the state transition probability (i.e. p(s' | s, a)).
         * 
         * @param state the state 
         * @param action the action
         * @param next_state the next state
         * @param t the timestep
         * @return double the probability
         */
        virtual double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const = 0;

        /**
         * @brief Get the distribution over next states.
         * 
         * @param state the current state
         * @param action the current action
         * @param t the timestep
         * @return the distribution over next states
         */
        virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getNextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;
    };
} // namespace sdm
