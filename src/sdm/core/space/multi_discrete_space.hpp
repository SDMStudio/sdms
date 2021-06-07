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
#include <sdm/core/item.hpp>
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
     * @tparam std::shared_ptr<Item> The type of items in each sub-discrete space.
     */
    class MultiDiscreteSpace : public DiscreteSpace,
                               public Joint<std::shared_ptr<Space>>
    {
    public:
        using value_type = Joint<std::shared_ptr<Item>>;
        using iterator_type = DiscreteSpace::iterator_type;

        /**
         * @brief Instantiate a default discrete space (MultiDiscreteSpace class)
         */
        MultiDiscreteSpace();

        /**
         * @brief Instantiate a multi discrete space from the list its sub-spaces (as shared pointer).
         * 
         */
        MultiDiscreteSpace(const std::vector<std::shared_ptr<Space>> &sub_spaces, bool store_items = true);

        /**
         * @brief Instantiate a multi discrete space from the list its sub-spaces (as shared pointer).
         * 
         */
        MultiDiscreteSpace(const std::vector<std::vector<std::shared_ptr<Item>>> &values, bool store_items = true);

        /**
         * @brief Copy constructor
         * 
         * @param copy the space to be copied
         */
        MultiDiscreteSpace(const MultiDiscreteSpace &copy);

        /**
         * @brief Instantiate a multi discrete space using a list of dimensions (one by single space). This constructor is only available for classes where std::shared_ptr<Item> is an integer (long, int, short, etc).
         * 
         * @param num_items the number of items in each spaces. If {k_1, k_2} then {0, 1, ..., k_1 - 1} are possible items in first subspace and {0, 1, ..., k_2 - 1} in the second subspace.
         */
        template <bool TBool = std::is_integral<std::shared_ptr<Item>>::value>
        MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<std::shared_ptr<Item>>> &num_items);

        bool isStoringItems() const;
        void storeItems(bool store_items);

        /**
         * @brief   Get a specific index of an item for a given agent
         * @param   ag_id index of the agent
         * @param   item the item we want to get the index
         */
        number getItemIndex(number ag_id, const std::shared_ptr<Item> &item) const;

        /**
         * @brief Get a specific item from its index
         * 
         * @param index the index
         * @return a pointer on the item 
         */
        std::shared_ptr<Item> getItem(number index) const;

        /**
         * @brief   Get a specific item for a given agent (from its index)
         * @param   ag_id index of the agent
         * @param   item_index the index of the item we want to get
         */
        std::shared_ptr<Item> getItem(number ag_id, number item_index) const;

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
        std::shared_ptr<Space> getSpace(number index) const;

        template <bool TBool = std::is_integral<std::shared_ptr<Item>>::value>
        void setSpaces(const std::enable_if_t<TBool, std::vector<std::shared_ptr<Item>>> &num_items);
        void setSpaces(const std::vector<std::vector<std::shared_ptr<Item>>> &);
        void setSpaces(const std::vector<std::shared_ptr<Space>> &spaces);

        /*!
         * @brief Transform joint item to its index in the list of all joint items.
         * @param jitem the joint item we want to get the index
         * @return the corresponding index
         */
        number getJointItemIndex(std::shared_ptr<Joint<std::shared_ptr<Item>>> &jitem) const;
        // number getJointItemIndex(const std::vector<std::shared_ptr<Item>> &) const;

        /*!
         * @brief Get the corresponding joint item from its index.
         */
        std::shared_ptr<Item> getJointItem(number) const;

        /**
         * @brief Get all the joint values
         * 
         * @return the list of all possible joint items
         */
        std::vector<std::shared_ptr<Item>> getAll();

        std::string str() const;

        virtual std::shared_ptr<iterator_type> begin();
        virtual std::shared_ptr<iterator_type> end();

        MultiDiscreteSpace &operator=(const MultiDiscreteSpace &);
        bool operator==(const MultiDiscreteSpace &other) const;
        bool operator!=(const MultiDiscreteSpace &other) const;

        /**
         * @brief Verify is the multi discrete space contains the Joint<std::shared_ptr<Item>>;
         * 
         * @return true 
         * @return false 
         */
        bool contains(const std::shared_ptr<Item> &) const;

        friend std::ostream &operator<<(std::ostream &os, const MultiDiscreteSpace &sp)
        {
            os << sp.str();
            return os;
        }

    protected:
        using jitems_bimap = DiscreteSpace::items_bimap;
        using jitems_bimap_value = jitems_bimap::value_type;

        /**
        *  @brief Generates all joint items and maintains a bimap of indexes and corresponding pointers of joint items
        */
        void generateJointItems();
        bool isGenerated();

        /**
         * @brief Sets the number of joint items
         * 
         */
        void setNumJItems(number);

        inline std::shared_ptr<DiscreteSpace> cast(const std::shared_ptr<Space> &space) const;

        bool store_items_;
    };

} // namespace sdm
