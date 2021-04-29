/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <unordered_set>

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/matrix.hpp>

//!
//! \file     dynamics.hpp
//! \author   Jilles S. Dibangoye
//! \brief    dynamics class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for the dynamics model.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

    //!
    //! \class  dynamics  dynamics.hpp
    //!
    class ObservationDynamics
    {
    protected:
        //! \brief transition and observation matrices
        std::vector<Matrix> o_model;

        //! \brief dynamics model of the probabilities of state-observation pairs given state-action pairs.
        std::vector<std::vector<Matrix>> dynamics;

        //! \brief map from next-state, current action pairs to set of next observations
        std::unordered_map<number, std::unordered_map<number, std::unordered_set<number>>> successor_observations;

    public:
        ObservationDynamics();

        ObservationDynamics(ObservationDynamics &copy);

        //! \param    num_jactions Number of joint actions
        //! \param    num_jobservations Number of joint observations
        //! \param    num_states Number of states
        //! \brief    Instantiate a transition model
        ObservationDynamics(number, number, number);

        //! \fn       void initDynamics(number, number, number)
        //! \param    num_jactions Number of joint actions
        //! \param    num_jobservations Number of joint observations
        //! \param    num_states Number of states
        //! \brief    Inits the dynamics model
        void initDynamics(number, number, number);

        //! \fn       double getObservationProbability(action, observation, state) const
        //! \param    x a specific state
        //! \param    u a specific joint action
        //! \param    z a specific joint observation
        //! \param    y a specific state
        //! \brief    Returns probability
        //! \return   value
        double getObservationProbability(number, number, number, number) const;

        //! \fn       double setObservationProbability(action, observation, state, double)
        //! \param    u a specific joint action
        //! \param    z a specific joint observation
        //! \param    x a specific state
        //! \param    double proba
        //! \return   void
        void setObservationProbability(number, number, number, double);

        //! \fn       const std::unordered_set<observation>& getObservationSuccessors(action, state);
        //! \param    u a specific joint action
        //! \param    x a specific state
        //! \brief    Returns set of observations
        //! \return   const std::unordered_set<observation>&
        const std::unordered_set<number> &getObservationSuccessors(number, number);

        //! \fn       void setObservations(const std::vector<Matrix>&)
        //! \param    const std::vector<Matrix>& matrices of observations, one for each action.
        //! \brief    Sets probability transitions
        void setObservations(const std::vector<Matrix> &);

        //! \fn       const Matrix& getObservations(action)
        //! \param    u a specific joint action
        //! \brief    Returns matrix of probability observations for the pre-defined action.
        const Matrix &getObservations(number) const;

        //! \fn       value getDynamics(state, action, observation, state) const
        //! \param    s a specific state (timestep t)
        //! \param    a a specific joint action
        //! \param    o a specific joint observation
        //! \param    s_ a specific state (timestep t+1)
        //! \brief    Returns probability
        //! \return   value
        double getDynamics(number, number, number, number) const;

        //! \fn       void setDynamics(state, action, observation, state, value)
        //! \param    s a specific state (timestep t)
        //! \param    a a specific joint action
        //! \param    o a specific joint observation
        //! \param    s_ a specific state (timestep t+1)
        //! \param   value
        //! \brief    Sets probability
        void setDynamics(number, number, number, number, double);

        //! \fn       const matrix& getDynamics(action, observation) const
        //! \param    a a specific joint action
        //! \param    o a specific joint observation
        //! \brief    Returns transition matrix
        //! \return   const matrix&
        const Matrix &getDynamics(number, number) const;

        //! \fn       void setDynamics(action, observation, const matrix&)
        //! \param    a a specific joint action
        //! \param    o a specific joint observation
        //! \brief    Sets transition matrix
        void setDynamics(number, number, const Matrix &);

        const std::vector<std::vector<Matrix>> &getDynamics() const;
        const std::vector<Matrix> &getObservationProbabilities() const;
    };
} // namespace sdm
