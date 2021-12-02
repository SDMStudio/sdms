/**
 * @file hierarchical_mpomdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file to transforme a MPOMDP in a hierarchical MPOMDP
 * @version 1.0
 * @date 27/07/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
    class TransformedMPOMDP : public MPOMDPInterface
    {
    public:
        TransformedMPOMDP(const std::shared_ptr<MPOMDPInterface> &mpomdp);

        virtual ~TransformedMPOMDP() {}

        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        number getNumAgents() const;

        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        number getHorizon() const;

        /**
         * @brief Get the discount factor at timestep t.
         * 
         * @param t the timestep
         * @return the discount factor
         */
        double getDiscount(number t) const;

        /**
         * @brief Get the initial distribution over states.
         * 
         * @return the initial distribution over states
         */
        std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const;

        /**
         * @brief Get ths state space at timestep t.
         * 
         * @param t the timestep
         * @return the state space
         */
        std::shared_ptr<Space> getStateSpace(number t) const;

        /**
         * @brief Get the reward at timestep t when executing an action in a specific state.
         * 
         * @param state the current state
         * @param action the action
         * @param t the timestep
         * @return double the reward for each agent
         */
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;

        double getMinReward(number t) const;

        double getMaxReward(number t) const;

        /**
         * @brief Get the transition probability, i.e. p(s' | s, a).
         * 
         * @param state the current state
         * @param action the action
         * @param next_state the next state
         * @param t the timestep
         * @return the probability
         */
        double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const;

        /**
         * @brief Get reachable states
         * 
         * @param state the current state
         * @param action the current action
         * @return the set of reachable states
         */
        std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;

        /**
         * @brief Get reachable observations
         * 
         * @param state the current state
         * @param action the current action
         * @return the set of reachable observations
         */
        std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const;

        /**
         * @brief Get the observation probability, i.e. p(o | s', a)
         * 
         * @param action the action
         * @param next_state the next state
         * @param observation the observation
         * @param t the timestep
         * @return the probability
         */
        double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

        /**
         * @brief Get the dynamics, i.e. p(s', o | s, a)
         * 
         * @param state the state at timestep t
         * @param action the action 
         * @param next_state the next state, i.e. timestep t+1
         * @param observation the observation
         * @param t the timestep
         * @return the probability
         */
        double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

        /**
         * @brief Get ths action space of agent i at timestep t.
         * 
         * @param agent_id the identifier of the agent 
         * @param t the timestep
         * @return the action space
         */
        std::shared_ptr<Space> getActionSpace(number agent_id, number t) const;

        /**
         * @brief Get ths action space at timestep t.
         * 
         * @param t the timestep
         * @return the action space
         */
        std::shared_ptr<Space> getActionSpace(number t) const;

        /**
         * @brief Get the action space.
         * @param observation the observation in consideration
         * @param t time step
         * @return the action space. 
         */
        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &observation, number t);

        /**
         * @brief Get ths observation space of agent i at timestep t.
         * 
         * @param agent_id the identifier of the agent 
         * @param t the timestep
         * @return the observation space
         */
        std::shared_ptr<Space> getObservationSpace(number agent_id, number t) const;

        /**
         * @brief Get ths observation space at timestep t.
         * 
         * @param t the timestep
         * @return the observation space
         */
        std::shared_ptr<Space> getObservationSpace(number t) const;

        void setInternalState(std::shared_ptr<State> state);
        
        std::shared_ptr<State> getInternalState() const;

        /**
         * @brief Get random action.
         * @param observation the observation in consideration.
         * @param t time step.
         * @return the random action. 
         */
        std::shared_ptr<Action> getRandomAction(const std::shared_ptr<State> &observation, number t);

        /**
         * @brief Reset the environment and return initial observation.
         * @return the initial observation
         */
        std::shared_ptr<State> reset();

        /**
         * @brief Do a step on the environment.
         * @param action the action to execute
         * @return the information produced. Include : next observation, rewards, episode done  
         */
        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
        
        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action, bool increment_timestep);

    protected:
        /** @brief The underlying MPOMDP */
        std::shared_ptr<MPOMDPInterface> mpomdp_;
    };

} // namespace sdm