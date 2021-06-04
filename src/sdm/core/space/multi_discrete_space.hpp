/**
 * @file multi_discrete_space.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File for MultiDiscreteSpace class.
 * @version 1.0
 * @date 01/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>
#include <assert.h>
#include <boost/bimap.hpp>

#include <sdm/types.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/joint.hpp>

/**
 * @namespace sdm
 *
 * @brief Namespace grouping all tools required for sequential decision making.
 * 
 */
namespace sdm
{
    /**
     * @brief This class provide a way to instantiate multi discrete space (i.e. list of discrete spaces). Typically it is used to store a set of spaces, one by agent (i.e. action_spaces in POSG). This can be view as a set of discrete spaces or as a discrete space of joint items.
     * 
     * @tparam TItem The type of items in each sub-discrete space.
     */
    template <typename TItem>
    class MultiDiscreteSpace : public DiscreteSpace<std::shared_ptr<Joint<TItem>>>,
                               public Joint<std::shared_ptr<DiscreteSpace<TItem>>>
    {
    public:
        using value_type = Joint<TItem>;

        /**
         * @brief Instantiate a default discrete space (MultiDiscreteSpace class)
         */
        MultiDiscreteSpace();

        /**
         * @brief Instantiate a multi discrete space from the list its sub-spaces (as shared pointer).
         * 
         */
        MultiDiscreteSpace(const std::vector<std::shared_ptr<DiscreteSpace<TItem>>> &sub_spaces);

        /**
         * @brief Instantiate a multi discrete space from the list its sub-spaces (as shared pointer).
         * 
         */
        MultiDiscreteSpace(const std::vector<std::vector<TItem>> &values);

        /**
         * @brief Cpoy constructor
         */
        MultiDiscreteSpace(const MultiDiscreteSpace<TItem> &copy);

        /**
         * @brief Instantiate a multi discrete space from the list its sub-spaces (as objects).
         * 
         */
        MultiDiscreteSpace(const std::vector<DiscreteSpace<TItem>> &);

        /**
         * @brief Instantiate a multi discrete space using a list of dimensions (one by single space). This constructor is only available for classes where TItem is an integer (long, int, short, etc).
         * 
         * @param num_items the number of items in each spaces. If {k_1, k_2} then {0, 1, ..., k_1 - 1} are possible items in first subspace and {0, 1, ..., k_2 - 1} in the second subspace.
         */
        template <bool TBool = std::is_integral<TItem>::value>
        MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<TItem>> &num_items);

        /**
         * @brief Transform joint item to single one
         * @param  jitem the joint item as a list of single items
         * @return the same joint item (as a single index to refer it)
         */
        number joint2single(const std::vector<TItem> &jitem) const;

        /**
         * @brief Transform single item to joint one
         * @param  sitem the index of a specific joint item
         * @return the same joint item (as a list of single items)
         */
        std::vector<TItem> single2joint(number sitem) const;

        /**
         * @brief Get the number of joint items
         */
        number getNumJointItems() const;

        /**
         * @brief   Get a specific index of an item for a given agent
         * @param   ag_id index of the agent
         * @param   item the item we want to get the index
         */
        number getItemIndex(number ag_id, const TItem &item) const;

        TItem getItem(number) const;

        /**
         * @brief   Get a specific item for a given agent (from its index)
         * @param   ag_id index of the agent
         * @param   item_index the index of the item we want to get
         */
        TItem getItem(number ag_id, number item_index) const;

        template <bool TBool = std::is_integral<TItem>::value>
        void setSpaces(const std::enable_if_t<TBool, std::vector<TItem>> &num_items);
        void setSpaces(const std::vector<std::vector<TItem>> &);
        void setSpaces(const std::vector<std::shared_ptr<DiscreteSpace<TItem>>> &spaces);
        void setSpaces(const std::vector<DiscreteSpace<TItem>> &spaces);

        /*!
         * @brief Transform joint item to its index in the list of all joint items.
         * @param jitem the joint item we want to get the index
         * @return the corresponding index
         */
        number getJointItemIndex(std::shared_ptr<Joint<TItem>> &jitem) const;
        number getJointItemIndex(const std::vector<TItem> &) const;

        /*!
         * @brief Get the corresponding joint item from its index.
         */
        std::shared_ptr<Joint<TItem>> getJointItem(number) const;

        /**
         * @brief Get all the joint values
         * 
         * @return the list of all possible joint items
         */
        std::vector<std::shared_ptr<Joint<TItem>>> getAll() const;

        std::string str() const;

        MultiDiscreteSpace<TItem> &operator=(const MultiDiscreteSpace<TItem> &);
        bool operator==(const MultiDiscreteSpace &other) const;
        bool operator!=(const MultiDiscreteSpace &other) const;

        /**
         * @brief Verify is the multi discrete space contains the Joint<TItem>;
         * 
         * @return true 
         * @return false 
         */
        bool contains(const std::vector<TItem> &) const;

        friend std::ostream &operator<<(std::ostream &os, const MultiDiscreteSpace<TItem> &sp)
        {
            os << sp.str();
            return os;
        }

    protected:
        typedef boost::bimaps::bimap<number, Joint<TItem>> jitems_bimap;
        typedef typename jitems_bimap::value_type jitems_bimap_value;

        /**
        *  @brief Generates all joint items and maintains a bimap of indexes and corresponding pointers of joint items
        */
        void generateJointItems();

        /**
         * @brief Sets the number of joint items
         * 
         */
        void setNumJItems(number);

        /**
         * @brief Get the number of sub-space.
         */
        number getNumSpaces() const;

        /**
         * @brief Get a specific subspace
         * 
         * @param index the index of the space
         * @return a shared pointer on a specific space 
         */
        std::shared_ptr<DiscreteSpace<TItem>> getSpace(number index) const;

        std::vector<std::shared_ptr<DiscreteSpace<TItem>>> spaces_;
    };

} // namespace sdm

#include <sdm/core/space/multi_discrete_space.tpp>
