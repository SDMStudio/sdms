/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/__constraint__.hpp>
#include <sdm/public/world.hpp>

#include <sdm/core/__state__.hpp>
#include <sdm/core/__agent__.hpp>
#include <sdm/core/__action__.hpp>
#include <sdm/core/__observation__.hpp>
#include <sdm/core/__reward__.hpp>
#include <sdm/core/__dynamics__.hpp>

#include <unordered_map>

//!
//! \file     __dmdp__.hpp
//! \author   Jilles S. Dibangoye
//! \brief    __dmdp__ class
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
  //! \class  __dmdp__.hpp
  //!
  class __dmdp__ : public World, public __constraint__
  {
  protected:
    //! \brief description of the Dec-POMDP's states.
    __state__ state_space_;

    //! \brief description of the Dec-POMDP's agents.
    __agent__ agent_space_;

    //! \brief description of the Dec-POMDP's actions.
    __action__ action_space_;

    //! \brief description of the Dec-POMDP's observations.
    __observation__ observation_space_;

    //! \brief description of the Dec-POMDP's reward function.
    __reward__ reward_;

    //! \brief description of the Dec-POMDP's dynamics function.
    __dynamics__ dynamics_;

    //! checked;
    bool isDmdp = false;
    bool isCheckedDmdp = false;

    //! \param state2observation is a surjective function from states to joint observations
    std::unordered_map<state, observation> state2observation;

  public:
    __dmdp__();

    //! \fn       bool isJointlyFullyObservable()
    //! \brief    Returns true if the dpomdp is jointly fully observable, and false otherwise.
    bool isJointlyFullyObservable();

    //! \fn       observation getObservationFromState(state);
    //! \brief    Returns observation given a corresponding state.
    observation getObservationFromState(state);

    agent getNumAgents() const;
    state getNumStates() const;
    observation getNumObservations() const;
    action getNumActions() const;

    __state__ getStateSpace() const;
    void setStateSpace(__state__);

    __agent__ getAgentSpace() const;
    void setAgentSpace(__agent__);

    __action__ getActionSpace() const;
    void setActionSpace(__action__);

    __observation__ getObservationSpace() const;
    void setObservationSpace(__observation__);

    __reward__ getReward() const;
    void setReward(__reward__);

    __dynamics__ getDynamics() const;
    void setDynamics(__dynamics__);
  };
} // namespace sdm
