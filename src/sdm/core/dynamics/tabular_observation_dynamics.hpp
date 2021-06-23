/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <unordered_set>

#include <sdm/types.hpp>
#include <sdm/core/dynamics/observation_dynamics_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
    //!
    //! \class  dynamics  dynamics.hpp
    //!
    class TabularObservationDynamics : public ObservationDynamicsInterface
    {
    public:
        /**
         * @brief Get the observation probability
         * 
         * @param state a specific state (timestep t)
         * @param action a specific action
         * @param obs a specific observation
         * @param next_state a specific state (timestep t+1)
         * @return double a probability
         */
        virtual double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t = 0) const = 0;

        /**
         * @brief Get the observation vector for a given action and next state 
         * 
         * @param action the action
         * @return the observation matrix
         */
        virtual const MappedVector<std::shared_ptr<Observation>> &getObservationProbabilities(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const = 0;

        /**
         * @brief Set the observation probability
         * 
         * @param action a specific action
         * @param observation a specific observation
         * @param next_state a specific state
         * @param proba a probability
         */
        virtual void setObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba) = 0;

        /**
         * @brief Set the observation probability
         * 
         * @param action 
         * @param next_state 
         * @param observation_probas 
         */
        virtual void setObservationProbabilities(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const MappedVector<std::shared_ptr<Observation>> &observation_probas) = 0;

        /**
         * @brief Get reachable observations from a state, suppose a specific action was executed. 
         * 
         * @param state the current state
         * @param action the current action
         * @param t the timestep
         * @return the list of next reachable observations 
         */
        virtual std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const = 0 ;

        /**
         * @brief Set the the state / observation transition probability (i.e. p(s', o | s, a)).
         * 
         * @param state the state 
         * @param action the action
         * @param next_state the next state
         * @param observation the observation
         * @param proba the probability
         * @param t the timestep
         * @return double the probability
         */
        virtual void setReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t = 0) = 0;

        /**
         * @brief Get the distribution over next observations.
         * 
         * @param state the state
         * @param action the action
         * @param next_state the next state
         * @param t the timestep
         * @return the distribution over observations
         */
        virtual std::shared_ptr<Distribution<std::shared_ptr<Observation>>> getNextObservationDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0);
    };
} // namespace sdm

