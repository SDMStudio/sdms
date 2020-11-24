/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <cstddef>
#include <vector>

#include <boost/bimap.hpp>

#include <sdm/types.hpp>
#include <sdm/utils/decision_rules/joint.hpp>


//!
//! \file     state.hpp
//! \author   Jilles S. Dibangoye
//! \brief    dpomdp class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for states.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  //!
  //! \class  state  state.hpp
  //! \brief refers to the state space
  class __state__
  {
  private:
  protected:
    //! \brief number of states in the system.
    state num_states_;

    //! \brief list of state names.
    bimap state_names_bimap;

  public:

    //! \fn     __state__()
    //! \brief  instantiate a default state space (__state__ class) 
    //! \return instance of __state__
    __state__();

    //! \fn     __state__(state num_states)
    //! \brief  instantiate a state space (__state__ class) 
    //! \param  num_states number of states.
    //! \return instance of __state__
    __state__(state);


    //! \fn     __state__(const std::vector<std::string> & state_names)
    //! \brief  instantiate a state space (__state__ class) 
    //! \param  state_names a list of state names.
    //! \return instance of __state__
    __state__(const std::vector<std::string> &);

    //! \fn       state getNumStates() const
    //! \brief    Returns the number of states
    //! \return   state number
    state getNumStates() const;

    //! \fn       void setNumStates(state)
    //! \brief    Sets the number of states
    void setNumStates(state);

    //! \fn       void setStatesNames(std::vector<std::string>&)
    //! \param    const std::vector<std::string>&
    //! \brief    Sets the names of states.
    void setStatesNames(const std::vector<std::string> &);

    //! \fn       state getStateIndex(const std::string&)
    //! \param    const std::string& a state name
    //! \brief    Returns the index of the state
    state getStateIndex(const std::string &) const;

    //! \fn       std::string getStateName(state)
    //! \param    state  index
    //! \brief    Returns the name associated with the state index
    std::string getStateName(state) const;
  };
} // namespace sdm
