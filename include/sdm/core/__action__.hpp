/*=============================================================================
  Copyright (c) 2016 Jilles Steeve Dibangoye
==============================================================================*/
#pragma once

#include <cstddef>
#include <vector>
#include <string>

#include <boost/bimap.hpp>

#include <sdm/types.hpp>

#include <sdm/utils/decision_rules/joint_action.hpp>

//!
//! \file     action.hpp
//! \author   Jilles S. Dibangoye
//! \brief    action class
//! \version  1.0
//! \date     12 Avril 2016
//!
//! This class provides getter and setter methods for actions.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

  //!
  //! \class  dpomdp  dpomdp.hpp
  //!
  //! \param  value   primitive type for rewards and probabilities.
  //! \param  matrix  type used to encode matrices.
  //! \param  vector  type used to encode vectors.
  //!
  class __action__
  {
  protected:
    //! \brief list of action names for each agent.
    std::vector<bimap> action_names_bimap;

    //! \brief table of the number of actions of each agent.
    std::vector<action> number_actions;

    //! \brief number of joint actions of all agents as a whole.
    action number_jactions;

    /*!
     *  \fn  void generateJointActions()
     *  \param agent  number of agents involved
     *  \brief generates all joint actions and maintains a bimap of indexes and
     *         corresponding pointers of joint actions.
     */
    void generateJointActions(agent);

  public:

    ~__action__();

    //! \fn       action getNumActions() const
    //! \brief    Returns the number of joint actions
    //! \return   joint action number
    action getNumActions() const;

    //! \fn       void setNumActions(action)
    //! \param    joint action number
    //! \brief    Sets the number of joint actions
    void setNumActions(action);

    //! \fn       void setNumActions(const std::vector<std::vector<std::string>>&)
    //! \param    const std::vector<std::vector<std::string>>&
    //! \brief    Sets the number of actions and their corresponding names for each agent.
    void setNumActions(const std::vector<std::vector<std::string>> &);

    //! \fn       void setNumActions(const std::vector<action>&)
    //! \param    const std::vector<action>&
    //! \brief    Sets the number of actions for each agent.
    void setNumActions(const std::vector<action> &);

    //! \fn       action getNumActions(agent) const
    //! \param    agent
    //! \brief    Returns the number of individual actions of agent
    //! \return   individual action number
    action getNumActions(agent) const;

    //! \fn       void setNumActions(agent, action)
    //! \param    agent ID
    //! \param    individual action number
    //! \brief    Sets the number of individual action of agent
    void setNumActions(agent, action);

    //! \fn       action getActionIndex(const std::string&)
    //! \param    agent   agent index
    //! \param    const std::string& individual action name
    //! \brief    Returns the index associated with the action name
    action getActionIndex(agent, const std::string &) const;

    action getActionIndex(agent, action);

    //! \fn       std::string getActionIndex(const std::string&)
    //! \param    agent   agent index
    //! \param    std::string individual action name
    //! \brief    Returns the index associated with the action name
    std::string getActionName(agent, action) const;

    //! \fn  action getJointActionIndex(joint_action*)
    //! \brief getter of the joint-action index
    //! \param joint_action*
    //! \return index the joint-action pointer
    action getJointActionIndex(joint_action *) const;

    //! \fn  action getJointActionIndex(std::vector<action>const &)
    //! \brief getter of the joint-action index
    //! \param std::vector<action>const &
    //! \return action = joint_action::getJointItemIdx(v);
    action getJointActionIndex(std::vector<action> const &) const;

    //! \fn  joint_action* getJointAction(action)
    //! \brief getter of the joint-action pointer
    //! \return const joint_action& the pointer to the joint-action index
    joint_action *getJointAction(action);
  };
} // namespace sdm
