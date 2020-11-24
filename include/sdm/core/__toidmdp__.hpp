/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/core/__dmdp__.hpp>

//!
//! \file     __toidmdp__.hpp
//! \author   Jilles S. Dibangoye
//! \brief    __toidmdp__ class
//! \version  1.0
//! \date     10 June 2018
//!
//! This class provides getter and setter methods for states.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  //!
  //! \class  __toidmdp__.hpp
  //!
  class __toidmdp__ : public __dmdp__
  {
  protected:
    //! \brief checked;
    bool isToidmdp = false, isCheckedToidmp = false;

    //! \brief agents' initial distributions
    std::unordered_map<agent, std::shared_ptr<Vector>> ibeliefs;

    //! \brief agents' dynamics
    std::unordered_map<agent, std::shared_ptr<__dynamics__>> idynamics;

    //! \brief is a bijective function from joint observations to states
    std::unordered_map<observation, state> observation2state;

  public:
    __toidmdp__();

    //! \fn       bool isTransitionIndependent()
    //! \brief    Returns true if the dmdp is transition independent, and false otherwise.
    bool isTransitionIndependent();

    //! \fn       state getStateFromObservation(observation);
    //! \brief    Returns state given the corresponding observation.
    state getStateFromObservation(observation);

    //! \fn         const std::shared_ptr<Vector>& getBelief(agent) const;
    //! \return     belief of agent
    const std::shared_ptr<Vector> &getBelief(agent) const;

    //! \fn       value getAgentDynamics(agent, observation, action, observation) const
    //! \param    agent
    //! \param    observation
    //! \param    action
    //! \param    observation
    //! \brief    Returns probability
    //! \return   probability
    double getAgentDynamics(agent, observation, action, observation) const;
  };
} // namespace sdm
