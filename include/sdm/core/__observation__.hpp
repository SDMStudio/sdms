/*=============================================================================
Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <sdm/types.hpp>

#include <sdm/utils/decision_rules/joint.hpp>
#include <sdm/utils/decision_rules/joint_observation.hpp>

//!
//! \file     observation.hpp
//! \author   Jilles S. Dibangoye
//! \brief    observation class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for all observations.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm{

  //!
  //! \class  observation  observation.hpp
  //!
  class __observation__ {
  protected:

    //! \param number_observations    table of the number of actions for each agent.
    std::vector<observation> number_observations;

    std::vector<bimap> observation_names_bimap;  //<! list of observation names for each agent.

    //! \param number_jobservations   number of joint observations of all agents as a whole.
    observation number_jobservations;

    /*!
    *  \fn  void generateJointObservations()
    *  \param  agent number of agents
    *  \brief generates all joint observations and maintains a bimap of indexes and
    *         corresponding pointers of joint observations.
    */
    void generateJointObservations(agent);

  public:

    ~__observation__();

    //! \fn       observation getNumObservations() const
    //! \brief    Returns the number of joint observation
    //! \return   joint observation number
    observation getNumObservations() const;

    //! \fn       void setNumObservations(observation)
    //! \param    joint observation number
    //! \brief    Sets the number of joint observations
    void setNumObservations(observation);

    //! \fn       void setNumActions(const std::vector<std::vector<std::string>>&)
    //! \param    const std::vector<std::vector<std::string>>&
    //! \brief    Sets the number of observations and their corresponding names for each agent.
    void setNumObservations(const std::vector<std::vector<std::string>>&);

    //! \fn       void setNumObservations(const std::vector<observation>&)
    //! \param    const std::vector<observation>&
    //! \brief    Sets the number of observations for each agent.
    void setNumObservations(const std::vector<observation>&);

    //! \fn       observation getNumObservations(agent) const
    //! \param    agent
    //! \brief    Returns the number of individual observations of agent
    //! \return   individual observation number
    observation getNumObservations(agent) const;

    //! \fn       void setNumObservations(agent, observation)
    //! \param    agent ID
    //! \param    individual observation number
    //! \brief    Sets the number of individual observation of agent
    void setNumObservations(agent, observation);

    //! \fn       observation getObservationIndex(agent, const std::string&)
    //! \param    agent   agent index
    //! \param    const std::string& individual observation name
    //! \brief    Returns the index associated with the observation name
    observation getObservationIndex(agent, const std::string&) const;

    observation getObservationIndex(agent, observation);

    //! \fn       const std::string& getActionName(agent, observation)
    //! \param    agent agent index
    //! \param    observation individual observation index
    //! \brief    Returns the name associated with the observation index
    std::string getObservationName(agent, observation) const;

    /*!
    * \fn  observation getJointObservaIndex(std::vector<observation>const &)
    * \brief getter of the joint-observation index
    * \param std::vector<observation>const &
    * \return index the joint-observation pointer
    */
    observation getJointObservationIndex(std::vector<observation>const &);

    /*!
     * \fn  action getJointObservationIndex(joint_observation*)
     * \brief getter of the joint_observation index
     * \param joint_observation*
     * \return index the joint_observation pointer
     */
    action getJointObservationIndex(joint_observation*);

    /*!
    * \fn  joint_observation* getJointObservation(observation)
    * \brief getter of the joint-observation pointer
    * \return joint_observation* the pointer to the joint-observation index
    */
    joint_observation* getJointObservation(observation);

  };
}
