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
namespace sdm{

  //!
  //! \class  dynamics  dynamics.hpp
  //!
  class __dynamics__ {
    protected:
      //! \brief transition and observation matrices
      std::vector<Matrix> t_model,  o_model;

      //! \brief dynamics model of the probabilities of state-observation pairs given state-action pairs.
      std::vector<std::vector<Matrix>> dynamics;

      //! \brief map from state, action pairs to set of next states
      std::unordered_map<state, std::unordered_map<action, std::unordered_set<state>>> successor_states;

      //! \brief map from next-state, current action pairs to set of next observations
      std::unordered_map<state, std::unordered_map<action, std::unordered_set<observation>>> successor_observations;

    public:

      __dynamics__();

      __dynamics__(action, observation, state);

      //! \fn       void initDynamics(action, observation)
      //! \param    action
      //! \param    observation
      //! \param    state
      //! \brief    Inits the dynamics model
      void initDynamics(action, observation, state);

      //! \fn       void setTransitionProbability(state, action, state, double, bool=false);
      //! \param    state
      //! \param    action
      //! \param    state
      //! \param    double prob
      //! \param    bool whether or not we cumulate probabilities.
      void setTransitionProbability(state, action, state, double, bool=false);

      //! \fn       double getTransitionProbability(state, action, state) const
      //! \param    state
      //! \param    action
      //! \param    state
      //! \brief    Returns probability
      //! \return   value
      double getTransitionProbability(state, action, state) const;

      //! \fn       const std::unordered_set<state>& getStateSuccessors(state, action);
      //! \param    state
      //! \param    action
      //! \brief    Returns set of next states
      //! \return   const std::unordered_set<state>&
      const std::unordered_set<state>& getStateSuccessors(state, action);

      //! \fn       void setTransitions(const std::vector<Matrix>&)
      //! \param    const std::vector<Matrix>& matrices of transitions, one for each action.
      //! \brief    Sets probability transitions
      void setTransitions(const std::vector<Matrix>&);

      //! \fn       const Matrix& getTransitions(action)
      //! \param    action
      //! \brief    Returns matrix of probability transitions for the pre-defined action.
      const Matrix& getTransitions(action);

      //! \fn       double getObservationProbability(action, observation, state) const
      //! \param    action
      //! \param    observation
      //! \param    state
      //! \brief    Returns probability
      //! \return   value
      double getObservationProbability(action, observation, state) const;

      //! \fn       double setObservationProbability(action, observation, state, double)
      //! \param    action
      //! \param    observation
      //! \param    state
      //! \param    double prob
      //! \return   void
      void setObservationProbability(action, observation, state, double);


      //! \fn       const std::unordered_set<observation>& getObservationSuccessors(action, state);
      //! \param    action
      //! \param    state
      //! \brief    Returns set of observations
      //! \return   const std::unordered_set<observation>&
      const std::unordered_set<observation>& getObservationSuccessors(action, state);

      //! \fn       void setObservations(const std::vector<Matrix>&)
      //! \param    const std::vector<Matrix>& matrices of observations, one for each action.
      //! \brief    Sets probability transitions
      void setObservations(const std::vector<Matrix>&);

      //! \fn       const Matrix& getObservations(action)
      //! \param    action
      //! \brief    Returns matrix of probability observations for the pre-defined action.
      const Matrix& getObservations(action);

      //! \fn       value getDynamics(state, action, observation, state) const
      //! \param    state
      //! \param    action
      //! \param    observation
      //! \param    state
      //! \brief    Returns probability
      //! \return   value
      double getDynamics(state, action, observation, state) const;

      //! \fn       void setDynamics(state, action, observation, state, value)
      //! \param    state
      //! \param    action
      //! \param    observation
      //! \param    state
      //! \param   value
      //! \brief    Sets probability
      void setDynamics(state, action, observation, state, double);

      //! \fn       const matrix& getDynamics(action, observation) const
      //! \param    action
      //! \param    observation
      //! \brief    Returns transition matrix
      //! \return   const matrix&
      const Matrix& getDynamics(action, observation) const;

      //! \fn       void setDynamics(action, observation, const matrix&)
      //! \param    action
      //! \param    observation
      //! \brief    Sets transition matrix
      void setDynamics(action, observation, const Matrix&);
  };
}
