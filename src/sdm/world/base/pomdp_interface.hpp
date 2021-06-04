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
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        virtual std::set<std::shared_ptr<Observation>> getAllObservations(number t) const = 0;

        /**
         * @brief Get the Reachablel Observations object
         * 
         * @param state 
         * @param action 
         * @param t 
         * @return std::set<std::shared_ptr<Observation>> 
         */
        virtual std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        /**
         * @brief Get the Obs Probability object
         * 
         * @param action 
         * @param next_state 
         * @param observation 
         * @param t 
         * @return double 
         */
        virtual double getObsProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const = 0;

        /**
         * @brief Get the Dynamics object
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param observation 
         * @param t 
         * @return double 
         */
        virtual double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const = 0;
    };
} // namespace sdm