/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <string>

#include <sdm/types.hpp>

//!
//! \file     agent.hpp
//! \author   Jilles S. Dibangoye
//! \brief    agent class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for agents.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  //!
  //! \class  agent  agents.hpp
  //!
  class __agent__
  {
  protected:
    //! \brief number of agents involved in the system.
    agent num_agents_;

    //! \brief mapping from names (of agent, actions, observations, and states) to indexes and vice-versa.
    bimap agent_names_bimap;

  public:

    //! \fn     __agent__()
    //! \brief  instantiate a default agent space (__agent__ class) 
    //! \return instance of __agent__
    __agent__();

    //! \fn     __agent__(agent num_agents)
    //! \brief  instantiate a agent space (__agent__ class) 
    //! \param  num_agents number of agents.
    //! \return instance of __agent__
    __agent__(agent);


    //! \fn     __agent__(const std::vector<std::string> & agent_names)
    //! \brief  instantiate a agent space (__agent__ class) 
    //! \param  agent_names a list of agent names.
    //! \return instance of __agent__
    __agent__(const std::vector<std::string> &);

    //! \fn       agent getNumAgents() const
    //! \brief    Returns the number of agents
    //! \return   agent number
    agent getNumAgents() const;

    //! \fn       void setNumAgents(agent)
    //! \brief    Sets the number of agents
    void setNumAgents(agent);

    //! \fn       void setNumAgents(std::vector<std::string>&)
    //! \param    const std::vector<std::string>&
    //! \brief    Sets the number of agents and their corresponding names.
    void setAgentsNames(const std::vector<std::string> &);

    //! \fn       agent getAgentIndex(const std::string&)
    //! \param    const std::string& a agent name
    //! \brief    Returns the index of the agent
    agent getAgentIndex(const std::string &);

    //! \fn       std::string getAgentName(agent);
    //! \param    agent  index
    //! \brief    Returns the name associated with the agent index
    std::string getAgentName(agent);
  };
} // namespace sdm
