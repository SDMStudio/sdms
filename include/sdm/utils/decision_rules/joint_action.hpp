/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye, Olivier Buffet, Charles Bessonet
==============================================================================*/
#pragma once

#include <vector>

#include <boost/bimap.hpp>

#include <sdm/utils/decision_rules/joint.hpp>

//!
//! \file     joint_action.hpp
//! \author   Jilles S. Dibangoye
//! \brief    joint_action class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for joint actions.
//!

/*!
 *  \namespace  sdm
 *  namespace   grouping all tools required for sequential decision making.
 */
namespace sdm
{
  /*!
   *  \class      joint_action      joint_action.hpp
   *  \brief      class of joint action instances.
   */
  class joint_action : public joint<action, 0>
  {
  public:
    /*!
      * \fn joint_action(agent, action*)
      * \param const std::vector<agent>&  table of agents
      * \param action* table of actions
      * \brief Constructor
      */
    joint_action(const std::vector<agent> &, const std::vector<action> &);

    /*!
      * \fn std::ostream& operator<<(std::ostream&, const joint_action&)
      * \brief print the joint action
      * \param std::ostream&
      * \param const joint_action& joint action to be printed
      * \return std::ostream&
      *
      * This method should produce an output of this form:
      * <joint-action id="1" />
      *   <agent id="0" action="open-left"/>
      *   <agent id="0" action="open-left"/>
      * </joint-action>
      */
    friend std::ostream &operator<<(std::ostream &, const joint_action &);
  };
} // namespace sdm
