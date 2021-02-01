/*=============================================================================
  Copyright (c) 2020 David Albert
==============================================================================*/
#pragma once

#include <vector>
#include <assert.h>
#include <boost/bimap.hpp>

#include <sdm/types.hpp>
#include <sdm/core/space/multi_space.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/joint.hpp>

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
    template <typename TItem>
    class MultiDiscreteSpace : public virtual MultiSpace<DiscreteSpace<TItem>>, public DiscreteSpace<Joint<TItem>>
    {
    protected:
        typedef boost::bimaps::bimap<number, Joint<TItem>> jitems_bimap;
        typedef typename jitems_bimap::value_type jitems_bimap_value;

        //! \brief Number of joint item
        // number num_jitem;

        //! \brief Mapping of joint and single items
        // jitems_bimap joint_items_bimap;

        /**
        *  \fn  void generateJointActions(number num_agents)
        *  \param num_agents  number of agents involved
        *  \brief generates all joint items and maintains a bimap of indexes and
        *         corresponding pointers of joint items.
        */
        void generateJointItems();

        //! \fn       void setNumJItems(number)
        //! \param    num_je number of joint items
        //! \brief    Sets the number of joint items
        void setNumJItems(number);

    public:
        using value_type = Joint<TItem>;

        //! \fn     MultiDiscreteSpace()
        //! \brief  instantiate a default discrete space (MultiDiscreteSpace class)
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace();

        //! \fn     MultiDiscreteSpace(const std::vector<number> & num_values)
        //! \brief  instantiate a multi discrete space (MultiDiscreteSpace class)
        //! \param  num_values the number of possible values for each agent.
        //! \return instance of MultiDiscreteSpace
        // MultiDiscreteSpace(const std::vector<number> &);

        //! \fn     MultiDiscreteSpace(const std::vector<std::vector<TItem>> & v_names)
        //! \brief  instantiate a multi discrete space (MultiDiscreteSpace class)
        //! \param  v_names a list of value names for each agent.
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace(const std::vector<std::vector<TItem>> &);

        //! \fn     MultiDiscreteSpace(const std::vector<DiscreteSpace> & spaces)
        //! \brief  instantiate a multi discrete space (MultiDiscreteSpace class)
        //! \param  spaces a list of value names for each agent.
        //! \return instance of MultiDiscreteSpace
        MultiDiscreteSpace(const std::vector<std::shared_ptr<DiscreteSpace<TItem>>> &);

        MultiDiscreteSpace(const MultiDiscreteSpace<TItem> &copy);

        MultiDiscreteSpace(const std::vector<DiscreteSpace<TItem>> &);

        //! \fn     number joint2single(const std::vector<number>& jItem)
        //! \brief  Transform joint item to single one
        //! \param  jItem a specific joint item
        //! \return the corresponding single item
        number joint2single(const std::vector<TItem> &) const;

        //! \fn     const std::vector<number>& single2joint(number jItem)
        //! \brief  Transform a joint item represented as a single one into the initial joint item
        //! \param  sItem a single item
        //! \return the corresponding joint item
        std::vector<TItem> single2joint(number) const;

        //! \fn       number getNumJItems() const
        //! \brief    Returns the number of joint items
        //! \return   number of joint items
        number getNumJointItems() const;

        //! \fn       number getItemIndex(number, const TItem &) const
        //! \brief    Returns the number of items for a given agent
        //! \param    ag_id index of the agent
        //! \param    name  item name
        //! \return   the index associated with the item name (for a given agent)
        number getItemIndex(number, const TItem &) const;

        TItem getItem(number) const;

        //! \fn       TItem getItemName(number ag_id, number action_id)
        //! \param    ag_id agent index
        //! \param    action_id individual action index
        //! \brief    the name associated with the action index
        TItem getItem(number, number) const;

        // void setSpaces(const std::vector<number> &);
        void setSpaces(const std::vector<std::vector<TItem>> &);
        void setSpaces(const std::vector<std::shared_ptr<DiscreteSpace<TItem>>> &spaces);
        void setSpaces(const std::vector<DiscreteSpace<TItem>> &spaces);

        /*!
       * \fn  number getJointItemIndex(JointItem*)
       * \brief Getter of the joint-item index
       * \param jitem pointer of JointItem
       * \return the corresponding index
       */
        number getJointItemIndex(Joint<TItem> &) const;

        /*!
         * \fn  number getJointItemIndex(JointItem*)
         * \brief Getter of the joint-item index
         * \param jitem a joint item as vector
         * \return the corresponding index
         */
        number getJointItemIndex(const std::vector<TItem> &) const;

        /*!
       * \fn  JointItem getJointItem(number)
       * \brief getter of the joint-item
       * \return the joint item given his index
       */
        Joint<TItem> getJointItem(number) const;

        std::vector<Joint<TItem>> getAll();

        std::string str() const;

        MultiDiscreteSpace<TItem> &operator=(const MultiDiscreteSpace<TItem> &);

        friend std::ostream &operator<<(std::ostream &os, const MultiDiscreteSpace<TItem> &sp)
        {
            os << sp.str();
            return os;
        }
    };
} // namespace sdm

#include <sdm/core/space/multi_discrete_space.tpp>