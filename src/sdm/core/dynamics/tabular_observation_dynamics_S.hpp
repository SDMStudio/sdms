/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <unordered_set>

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/core/dynamics/tabular_observation_dynamics.hpp>


namespace sdm
{
    //!
    //! \class  dynamics  dynamics.hpp
    //!
    class TabularObservationDynamicsS : public TabularObservationDynamics
    {
    public:
        TabularObservationDynamicsS();

        TabularObservationDynamicsS(const TabularObservationDynamicsS &copy);
        
        virtual ~TabularObservationDynamicsS();

        /**
         * @brief Get the observation probability
         * 
         * @param state a specific state (timestep t)
         * @param action a specific action
         * @param obs a specific observation
         * @param next_state a specific state (timestep t+1)
         * @return double a probability
         */
        double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t = 0) const;

        /**
         * @brief Get the observation vector for a given action and next state 
         * 
         * @param action the action
         * @return the observation matrix
         */
        const MappedVector<std::shared_ptr<Observation>> &getObservationProbabilities(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;

        /**
         * @brief Set the observation probability
         * 
         * @param action a specific action
         * @param observation a specific observation
         * @param next_state a specific state
         * @param proba a probability
         */
        void setObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, double proba);

        /**
         * @brief Set the observation probability
         * 
         * @param action 
         * @param next_state 
         * @param observation_probas 
         */
        void setObservationProbabilities(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const MappedVector<std::shared_ptr<Observation>> &observation_probas);

        void setObservationModel(const MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>> &o_model);

        /**
         * @brief Get reachable observations from a state, suppose a specific action was executed. 
         * 
         * @param state the current state
         * @param action the current action
         * @param t the timestep
         * @return the list of next reachable observations 
         */
        std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;

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
        void setReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t = 0);

    protected:
        //! \brief transition and observation matrices
        MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Observation>> observation_model_;
        
        //! \brief map from next-state to set of next observations
        std::unordered_map<std::shared_ptr<State>, std::set<std::shared_ptr<Observation>>> successor_observations_;
    };
} // namespace sdm
