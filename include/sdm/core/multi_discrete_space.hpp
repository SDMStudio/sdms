/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <boost/bimap.hpp>

#include <sdm/core/space.hpp>
#include <sdm/core/discrete_space.hpp>
#include <sdm/utils/decision_rules/joint.hpp>
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
        typedef boost::bimaps::bimap<JointItem, number> jitems_bimap;
        typedef typename jitems_bimap::value_type jitems2index;

        //! \brief list of action names for each agent.
        std::vector<DiscreteSpace> spaces_;

        //! \brief list of action names for each agent.
        std::vector<std::string> names_;

        //! \brief Number of joint element
        number num_jelement;

        //! \brief Mapping of joint and single items
        jitems_bimap joint_items_bimap;

        /**
        *  \fn  void generateJointActions(number num_agents)
        *  \param num_agents  number of agents involved
        *  \brief generates all joint elements and maintains a bimap of indexes and
        *         corresponding pointers of joint elements.
        */
        void generateJointElements();

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

        std::vector<number> getDim() const;

        //! \fn     std::vector<number> sample()
        //! \brief  Sample an element from the space
        //! \return uniformly sampled element of the space
        std::vector<number> sample() const;

        //! \fn     number joint2single(const std::vector<number>& jelement)
        //! \brief  Transform joint element to single one
        //! \param  jelement a specific joint element
        //! \return the corresponding single element
        number joint2single(const std::vector<number> &) const;

        //! \fn     const std::vector<number>& single2joint(number jelement)
        //! \brief  Transform a joint element represented as a single one into the initial joint element
        //! \param  selement a single element
        //! \return the corresponding joint element
        const std::vector<number> &single2joint(number) const;

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
        //! \return   the index associated with the element name (for a given agent)
        number getElementIndex(number, const std::string &) const;

        //! \fn       std::string getElementName(number ag_id, number action_id)
        //! \param    ag_id agent index
        //! \param    action_id individual action index
        //! \brief    the name associated with the action index
        std::string getElementName(number, number) const;

        //! \fn       std::string getElementName(number ag_id)
        //! \param    ag_id agent index
        //! \brief    the name of the n ieme space
        std::string getElementName(number) const;

        void setNames(const std::vector<std::string> &);

        void setSpaces(const std::vector<number> &);

        void setSpaces(const std::vector<int> &);

        void setSpaces(const std::vector<std::vector<std::string>> &);

        void setSpaces(const std::vector<DiscreteSpace> &);

        /*!
       * \fn  number getJointElementIndex(JointItem*)
       * \brief Getter of the joint-element index
       * \param jitem pointer of JointItem
       * \return the corresponding index
       */
        number getJointElementIndex(JointItem *) const;

        /*!
         * \fn  number getJointElementIndex(JointItem*)
         * \brief Getter of the joint-element index
         * \param jitem a joint item as vector
         * \return the corresponding index
         */
        number getJointElementIndex(std::vector<number> const &) const;

        /*!
       * \fn  JointElement getJointElement(number)
       * \brief getter of the joint-item
       * \return the joint item given his index
       */
        const JointItem &getJointElement(number) const;

        std::string str() const;

        MultiDiscreteSpace &operator=(const MultiDiscreteSpace &);
        bool operator==(const MultiDiscreteSpace &) const;
        bool operator!=(const MultiDiscreteSpace &) const;
        friend std::ostream &operator<<(std::ostream &os, const MultiDiscreteSpace &sp);
    };
} // namespace sdm
