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
#include <sdm/utils/decision_rules/joint.hpp>

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
    //! \class  MDSpace
    template <typename TItem>
    class MDSpace : public virtual MultiSpace<DSpace<TItem>>, public DSpace<Joint<TItem>>
    {
    protected:
        typedef boost::bimaps::bimap<Joint<TItem>, number> jitems_bimap;
        typedef typename jitems_bimap::value_type jitems2index;

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
        //! \fn     MDSpace()
        //! \brief  instantiate a default discrete space (MDSpace class)
        //! \return instance of MDSpace
        MDSpace();

        //! \fn     MDSpace(const std::vector<number> & num_values)
        //! \brief  instantiate a multi discrete space (MDSpace class)
        //! \param  num_values the number of possible values for each agent.
        //! \return instance of MDSpace
        MDSpace(const std::vector<number> &);

        //! \fn     MDSpace(const std::vector<std::vector<TItem>> & v_names)
        //! \brief  instantiate a multi discrete space (MDSpace class)
        //! \param  v_names a list of value names for each agent.
        //! \return instance of MDSpace
        MDSpace(const std::vector<std::vector<TItem>> &);

        //! \fn     MDSpace(const std::vector<DSpace> & spaces)
        //! \brief  instantiate a multi discrete space (MDSpace class)
        //! \param  spaces a list of value names for each agent.
        //! \return instance of MDSpace
        MDSpace(const std::vector<std::shared_ptr<DSpace<TItem>>> &);

        //! \fn     number joint2single(const std::vector<number>& jelement)
        //! \brief  Transform joint element to single one
        //! \param  jelement a specific joint element
        //! \return the corresponding single element
        number joint2single(const std::vector<TItem> &) const;

        //! \fn     const std::vector<number>& single2joint(number jelement)
        //! \brief  Transform a joint element represented as a single one into the initial joint element
        //! \param  selement a single element
        //! \return the corresponding joint element
        const std::vector<TItem> &single2joint(number) const;

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

        //! \fn       number getElementIndex(number, const TItem &) const
        //! \brief    Returns the number of elements for a given agent
        //! \param    ag_id index of the agent
        //! \param    name  element name
        //! \return   the index associated with the element name (for a given agent)
        number getElementIndex(number, const TItem &) const;

        //! \fn       TItem getElementName(number ag_id, number action_id)
        //! \param    ag_id agent index
        //! \param    action_id individual action index
        //! \brief    the name associated with the action index
        TItem getElement(number, number) const;

        void setSpaces(const std::vector<number> &);
        void setSpaces(const std::vector<int> &);
        void setSpaces(const std::vector<std::vector<TItem>> &);
        void setSpaces(const std::vector<std::shared_ptr<DSpace<TItem>>> &spaces);

        /*!
       * \fn  number getJointElementIndex(JointItem*)
       * \brief Getter of the joint-element index
       * \param jitem pointer of JointItem
       * \return the corresponding index
       */
        number getJointElementIndex(Joint<TItem> &) const;

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
        const Joint<TItem> &getJointElement(number) const;

        std::string str() const;
        MDSpace<TItem> &operator=(const MDSpace<TItem> &);
        friend std::ostream &operator<<(std::ostream &os, const MDSpace<TItem> &sp)
        {
            os << sp.str();
            return os;
        }

        std::vector<Joint<TItem>> getAll();

        // DSpace<Joint<TItem>>::iterator begin()
        // {
        //     if (this->all_items_.size() == 0)
        //     {
        //         std::vector<TItem> first_item;
        //         for (auto space : this->getSpaces())
        //         {
        //             first_item.push_back(*space->begin());
        //         }
        //         this->all_items_.push_back(Joint<TItem>());
        //     }
        //     return DSpace<Joint<TItem>>::begin();
        // }

        // DSpace<Joint<TItem>>::iterator end()
        // {
        //     return DSpace<Joint<TItem>>::end();
        // }

        // std::shared_ptr<Joint<TItem>> begin();
        // std::shared_ptr<Joint<TItem>> next();
        // std::shared_ptr<Joint<TItem>> end();

        // protected:
        //     class my_iterator
        //     {

        //     public:
        //         typedef my_iterator self_type;
        //         typedef Joint<TItem> value_type;
        //         typedef Joint<TItem> &reference;
        //         typedef std::shared_ptr<Joint<TItem>> pointer;
        //         typedef std::forward_iterator_tag iterator_category;
        //         typedef Joint<TItem> difference_type;

        //         explicit my_iterator(std::vector<typename DSpace<TItem>::iterator> sub_iterators) : sub_iterators_(sub_iterators)
        //         {
        //         }

        //         // iterator &operator++()
        //         // {
        //         //     self_type i = *this;
        //         //     // ptr_++;
        //         //     // return i;
        //         //     // num = TO >= FROM ? num + 1 : num - 1;
        //         //     return *this;
        //         // }
        //         // iterator operator++(int)
        //         // {
        //         //     iterator retval = *this;
        //         //     ++(*this);
        //         //     return retval;
        //         // }
        //         // bool operator==(iterator other) const
        //         // {
        //         //     for (int i = 0; i < sub_iterators_.size(); i++)
        //         //     {
        //         //         if (sub_iterators_[i] != other.sub_iterators_[i])
        //         //         {
        //         //             return false;
        //         //         }
        //         //     }
        //         //     return true;
        //         // }
        //         // bool operator!=(iterator other) const { return !(*this == other); }

        //         // reference &operator*() const
        //         // {
        //         //     return Joint<TItem>(this->compute_values());
        //         // }

        //         // pointer operator->()
        //         // {
        //         //     return std::shared_ptr<value_type>(new value_type(this->compute_values()));
        //         // }

        //     private:
        //         std::vector<typename DSpace<TItem>::iterator> sub_iterators_;

        //         std::vector<TItem> compute_values()
        //         {
        //             std::vector<TItem> values;
        //             for (auto it : sub_iterators_)
        //             {
        //                 values.push_back(*it);
        //             }
        //             return values;
        //         }
        //     };
    };
} // namespace sdm

#include <sdm/core/space/multi_discrete_space.tpp>