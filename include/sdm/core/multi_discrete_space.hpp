/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <boost/bimap.hpp>

#include <sdm/core/space.hpp>
#include <sdm/core/discrete_space.hpp>
#include <sdm/types.hpp>

//!
//! \file     multi_discrete_space.hpp
//! \author   David Albert
//! \brief    discrete space class
//! \version  1.0
//! \date     24 novembre 2020
//!
//! This class provide a way to instantiate multi discrete space (i.e. list of discrete spaces).
//! Typically it is used to store a set of spaces, one by agent (i.e. action_spaces in POSG).

//! \namespace  sdm
//! \brief Namespace grouping all tools required for sequential decision making.
namespace sdm
{
    //! \class  MultiDiscreteSpace
    class MultiDiscreteSpace : public Space
    {
    protected:
        //! \brief list of action names for each agent.
        std::vector<DiscreteSpace> spaces_;

        //! \brief list of action names for each agent.
        std::vector<std::string> names_;

        //! \brief Number of joint element
        number num_jelement;

        /**
        *  \fn  void generateJointActions(number num_agents)
        *  \param num_agents  number of agents involved
        *  \brief generates all joint elements and maintains a bimap of indexes and
        *         corresponding pointers of joint elements.
        */
        void generateJointElements(number);

        //! \fn       void setNumJElements(number)
        //! \param    num_je number of joint elements
        //! \brief    Sets the number of joint elements
        void setNumJElements(number);

    public:
        //! \fn     MultiDiscreteSpace()
        //! \brief  instantiate a default discrete space (MultiDiscreteSpace class)
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace();

        //! \fn     MultiDiscreteSpace(const std::vector<number> & num_values)
        //! \brief  instantiate a multi discrete space (MultiDiscreteSpace class)
        //! \param  num_values the number of possible values for each agent.
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace(const std::vector<number> &);

        //! \fn     MultiDiscreteSpace(const std::vector<int> & num_values)
        //! \brief  instantiate a multi discrete space (MultiDiscreteSpace class)
        //! \param  num_values the number of possible values for each agent.
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace(const std::vector<int> &);

        //! \fn     MultiDiscreteSpace(const std::vector<std::vector<std::string>> & v_names)
        //! \brief  instantiate a multi discrete space (MultiDiscreteSpace class)
        //! \param  v_names a list of value names for each agent.
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace(const std::vector<std::vector<std::string>> &);

        //! \fn     MultiDiscreteSpace(const std::vector<DiscreteSpace> & spaces)
        //! \brief  instantiate a multi discrete space (MultiDiscreteSpace class)
        //! \param  spaces a list of value names for each agent.
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace(const std::vector<DiscreteSpace> &);

        //! \fn     std::vector<number> sample()
        //! \brief  Sample an element from the space
        //! \return uniformly sampled element of the space
        std::vector<number> sample() const;

        //! \fn     number joint2single(const std::vector<number>& jelement)
        //! \brief  Transform joint element to single one
        //! \param  jelement a specific joint element
        //! \return the corresponding single element
        number joint2single(const std::vector<number>&) const;

        //! \fn     const std::vector<number>& single2joint(number jelement)
        //! \brief  Transform a joint element represented as a single one into the initial joint element
        //! \param  selement a single element
        //! \return the corresponding joint element
        const std::vector<number>& single2joint(number) const;
        
        //! \brief Getter
        number getNumSpaces() const;

        std::vector<DiscreteSpace> getSpaces() const;

        //! \fn       number getSpace(number)
        //! \param    agent_id id of the agent
        //! \brief    Gets an individual space from index
        const DiscreteSpace &getSpace(number) const;

        //! \fn       number getNumElements() const
        //! \brief    Returns the number of elements (i.e. alias to getNumSpaces)
        //! \return   number of spaces
        number getNumElements() const;
        
        //! \brief 
        number getNumElements(number idx) const;

        //! \fn       number getNumJElements() const
        //! \brief    Returns the number of joint elements
        //! \return   number of joint elements
        number getNumJElements() const;

        //! \fn       number getElementIndex(number, const std::string &) const
        //! \brief    Returns the number of elements for a given agent
        //! \param    ag_id index of the agent
        //! \param    name  element name
        //! \return   number of elements for a given agent
        number getElementIndex(number, const std::string &) const;

        std::string getElementName(number) const;

        std::string getElementName(number, number) const;

        //! \return  individual action from joint element
        // number getActionIndex(number ag, number jelement);

        // action getJointActionIndex(joint_action *) const;

        // number getJointActionIndex(std::vector<action> const &) const;

        //! \fn  joint_action* getJointAction(action)
        //! \brief getter of the joint-action pointer
        //! \return const joint_action& the pointer to the joint-action index
        // JointElement& getJointElement(action);

        void setNames(const std::vector<std::string> &);

        void setSpaces(const std::vector<number> &);

        void setSpaces(const std::vector<int> &);

        void setSpaces(const std::vector<std::vector<std::string>> &);

        void setSpaces(const std::vector<DiscreteSpace> &);

        MultiDiscreteSpace &operator=(const MultiDiscreteSpace &);
        bool operator==(const MultiDiscreteSpace &) const;
        bool operator!=(const MultiDiscreteSpace &) const;
        friend std::ostream &operator<<(std::ostream &os, const MultiDiscreteSpace &sp);
    };
} // namespace sdm
