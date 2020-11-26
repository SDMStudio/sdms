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
  class StateDynamics
  {
  protected:
    //! \brief transition and observation matrices
    std::vector<Matrix> t_model;

    //! \brief map from state, action pairs to set of next states
    std::unordered_map<state, std::unordered_map<action, std::unordered_set<state>>> successor_states;

  public:
    StateDynamics();

    //! \param    num_jactions Number of joint actions
    //! \param    num_states Number of states
    //! \brief    Instantiate a transition model
    StateDynamics(number, number);

    //! \fn       void initDynamics(number, number)
    //! \param    num_jactions Number of joint actions
    //! \param    num_states Number of states
    //! \brief    Inits the dynamics model
    void initDynamics(number, number);

    //! \fn       void setTransitionProbability(number, number, number, double, bool=false);
    //! \param    x A specific state (the state at timestep t)
    //! \param    jaction A specific joint action
    //! \param    y A specific state (the state at timestep t+1)
    //! \param    double probability of the transition
    //! \param    bool whether or not we cumulate probabilities.
    void setTransitionProbability(number, number, number, double, bool = false);

    //! \fn       double getTransitionProbability(number, number, number) const
    //! \param    x A specific state (the state at timestep t)
    //! \param    jaction A specific joint actioon
    //! \param    y A specific state (the state at timestep t+1)
    //! \brief    Get transition probability
    //! \return   the transition probability
    double getTransitionProbability(number, number, number) const;

    //! \fn       const std::unordered_set<number>& getStateSuccessors(number, number);
    //! \param    x A specific state
    //! \param    jaction A specific joint actioon
    //! \brief    Returns set of possible next states
    //! \return   const std::unordered_set<state>&
    const std::unordered_set<state> &getStateSuccessors(number, number);

    //! \fn       void setTransitions(const std::vector<Matrix>&)
    //! \param    t_model matrices of transitions, one for each action.
    //! \brief    Sets probability transitions
    void setTransitions(const std::vector<Matrix> &);

    //! \fn       const Matrix& getTransitions(number)
    //! \param    jaction A specific joint action
    //! \brief    Returns matrix of probability transitions for the pre-defined action.
    const Matrix &getTransitions(number);
  };
} // namespace sdm
