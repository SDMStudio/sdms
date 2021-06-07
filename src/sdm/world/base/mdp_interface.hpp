/**
 * @file discrete_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the MDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/distribution.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class MDPInterface
    {
    public:
        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        virtual number getNumAgents() const = 0;

        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        virtual number getHorizon() const = 0;

        /**
         * @brief Get the discount factor at timestep t.
         * 
         * @param t the timestep
         * @return the discount factor
         */
        virtual double getDiscount(number t) const = 0;

        /**
         * @brief Get the initial distribution over states.
         * 
         * @return the initial distribution over states
         */
        virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const = 0;

        /**
         * @brief Get all states
         * 
         * @return the set of states 
         */
        virtual std::vector<std::shared_ptr<State>> getAllStates(number t) const = 0;

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        virtual std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        /**
         * @brief Get all actions
         * 
         * @return the set of actions 
         */
        virtual std::vector<std::shared_ptr<Action>> getAllActions(number t) const = 0;

        /**
         * @brief Get the reward
         * 
         * @param state 
         * @param action 
         * @param t 
         * @return double 
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;
        
        virtual double getMinReward(number t) const = 0;
        
        virtual double getMaxReward(number t) const = 0;

        /**
         * @brief Get the Transition Probability object
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param t 
         * @return double 
         */
        virtual double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const = 0;
    };

} // namespace sdm