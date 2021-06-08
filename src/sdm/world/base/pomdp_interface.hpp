/**
 * @file pomdp_interface.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the POMDP class.
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
#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class POMDPInterface : public MDPInterface
    {
    public:

        /**
         * @brief Get ths observation space at timestep t.
         * 
         * @param t the timestep
         * @return the observation space
         */
        virtual std::shared_ptr<Space> getObservationSpace(number t) const = 0;

      
        /**
         * @brief Get reachable observations
         * 
         * @param state the current state
         * @param action the current action
         * @return the set of reachable observations
         */
        virtual std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        /**
         * @brief Get the observation probability, i.e. p(o | s', a)
         * 
         * @param action the action
         * @param next_state the next state
         * @param observation the observation
         * @param t the timestep
         * @return the probability
         */
        virtual double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const = 0;

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
        virtual double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const = 0;
    };
} // namespace sdm