/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye, Olivier Buffet, Charles Bessonet
==============================================================================*/
#pragma once

#include <sdm/utils/decision_rules/joint.hpp>

//!
//! \file     joint_observation.hpp
//! \author   Jilles S. Dibangoye
//! \brief    joint_observation class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for joint observations.
//!

/*!
 *  \namespace  sdm
 *  namespace   grouping all tools required for sequential decision making.
 */
namespace sdm{
  /*!
   *  \class      joint_observation      joint_observation.hpp
   *  \brief      class of joint observation instances.
   */
   class joint_observation : public joint<observation, 1>{
   public:
     /*!
      * \fn joint_observation(agent, observation*)
      * \param const std::vector<agent>& table of agents
      * \param const std::vector<observation>& table of observations
      * \brief Constructor
      */
     joint_observation(const std::vector<agent>&, const std::vector<observation>&);

     /*!
      * \fn std::ostream& operator<<(std::ostream&, const joint_observation&)
      * \brief print the joint observation
      * \param std::ostream&
      * \param const joint_observation& joint observation to be printed
      * \return std::ostream&
      *
      * This method should produce an output of this form:
      * <joint-observation id="1" />
      *   <agent id="0" observation="hear-left"/>
      *   <agent id="0" observation="hear-left"/>
      * </joint-observation>
      */
     friend std::ostream& operator<<(std::ostream&, const joint_observation&);
   };
}
