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

#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/serialized_mmdp.hpp>

#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>

namespace sdm
{
    class SerializedMPOMDP : public SerializedMMDP, public POMDPInterface
    {
    public:
        SerializedMPOMDP(std::shared_ptr<POMDPInterface> mpomdp);

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        virtual std::vector<std::shared_ptr<Observation>> getAllObservations(number t) const ;

        /**
         * @brief Get the Reachablel Observations object
         * 
         * @param state 
         * @param action 
         * @param next_state
         * @param t 
         * @return std::set<std::shared_ptr<Observation>> 
         */
        virtual std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action,const std::shared_ptr<State> &next_state, number t) const ;

        /**
         * @brief Get the Obs Probability object
         * 
         * @param state
         * @param action 
         * @param next_state 
         * @param observation 
         * @param t 
         * @return double 
         */
        virtual double getObsProbability(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

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
        virtual double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

    protected:

        std::shared_ptr<Observation> empty_serial_observation;

        /**
         * @brief Map (serial state, seial action, serial observation, serial state) to Set of reachable seial observation
         * 
         */
        // std::unordered_map<std::shared_ptr<State>, std::unordered_map<std::shared_ptr<Action>, std::unordered_map<std::shared_ptr<State>, std::set<std::shared_ptr<Observation>>>>> reachable_obs_state_space;
        std::shared_ptr<ObservationDynamicsInterface> obs_dynamics_;

        /**
         * @brief  Create the Observation Space
         * 
         */
        std::shared_ptr<MultiSpace<DiscreteSpace<std::shared_ptr<Observation>>>> serialized_observation_space_;

        /**
         * @brief Initialize Serial Observation Space
         * 
         */
        void setupObservationSpace(std::shared_ptr<POMDPInterface> mpomdp);

        /**
         * @brief Initialize "reachable_observation_space"
         * 
         */

        void setReachableObservationSpace(std::shared_ptr<POMDPInterface> mpomdp);

        /**
         * @brief Get the Pointeur object of a precise Joint Action
         * 
         * @return std::shared_ptr<Joint<std::shared_ptr<Action>>> 
         */
        const std::shared_ptr<Observation> getPointeurObservation(Joint<std::shared_ptr<Observation>> &) const;

    };

} // namespace sdm