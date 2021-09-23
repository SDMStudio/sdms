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
#include <sdm/core/space/space.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/world/gym_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class MDPInterface : virtual public GymInterface
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
         * @brief Get ths state space at timestep t.
         * 
         * @param t the timestep
         * @return the state space
         */
        virtual std::shared_ptr<Space> getStateSpace(number t) const = 0;

        /**
         * @brief Get ths action space at timestep t.
         * 
         * @param t the timestep
         * @return the action space
         */
        virtual std::shared_ptr<Space> getActionSpace(number t) const = 0;

        /**
         * @brief Get the reward at timestep t when executing an action in a specific state.
         * 
         * @param state the current state
         * @param action the action
         * @param t the timestep
         * @return double the reward for each agent
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;
        
        virtual double getMinReward(number t) const = 0;
        virtual double getMaxReward(number t) const = 0;

        virtual std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action) = 0;
        virtual std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action, bool increment_timestep) = 0;

        /**
         * @brief Get the transition probability, i.e. p(s' | s, a).
         * 
         * @param state the current state
         * @param action the action
         * @param next_state the next state
         * @param t the timestep
         * @return the probability
         */
        virtual double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const = 0;

        /**
         * @brief Get reachable states
         * 
         * @param state the current state
         * @param action the current action
         * @return the set of reachable states
         */
        virtual std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        virtual void setInternalState(std::shared_ptr<State> state) = 0;

        virtual std::shared_ptr<State> getInternalState() const = 0;
    };

} // namespace sdm