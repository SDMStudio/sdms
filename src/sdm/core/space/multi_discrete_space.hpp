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
     * @tparam TItem The type of items in each sub-discrete space.
     */
    template <typename TItem>
    class MultiDiscreteSpace : public DiscreteSpace<TItem>,
                               public Joint<std::shared_ptr<BaseSpace<TItem>>>
    {
    public:
        using value_type = Joint<TItem>;
        using iterator_type = DiscreteSpace<TItem>::iterator_type;

        /**
         * @brief Instantiate a default discrete space (MultiDiscreteSpace class)
         */
        MultiDiscreteSpace();

        /**
         * @brief Instantiate a multi discrete space from the list its sub-spaces (as shared pointer).
         *
         */
        MultiDiscreteSpace(const std::vector<std::shared_ptr<BaseSpace<TItem>>> &sub_spaces, bool store_items = true);

        /**
         * @brief Instantiate a multi discrete space from the list its sub-spaces (as shared pointer).
         *
         */
        MultiDiscreteSpace(const std::vector<std::vector<TItem>> &values, bool store_items = true);

        /**
         * @brief Copy constructor
         *
         * @param copy the space to be copied
         */
        MultiDiscreteSpace(const MultiDiscreteSpace &copy);

        /**
         * @brief Instantiate a multi discrete space using a list of dimensions (one by single space). This constructor is only available for classes where TItem is an integer (long, int, short, etc).
         *
         * @param num_items the number of items in each spaces. If {k_1, k_2} then {0, 1, ..., k_1 - 1} are possible items in first subspace and {0, 1, ..., k_2 - 1} in the second subspace.
         */
        template <bool TBool = std::is_integral<TItem>::value>
        MultiDiscreteSpace(const std::enable_if_t<TBool, std::vector<TItem>> &num_items);

        /**
         * @brief   Get a specific index of an item for a given agent
         * @param   ag_id index of the agent
         * @param   item the item we want to get the index
         */
        number getItemIndex(number ag_id, const TItem &item) const;

        /**
         * @brief Get a specific item from its index
         *
         * @param index the index
         * @return a pointer on the item
         */
        TItem getItem(number index) const;

        /**
         * @brief   Get a specific item for a given agent (from its index)
         * @param   ag_id index of the agent
         * @param   item_index the index of the item we want to get
         */
        TItem getItem(number ag_id, number item_index) const;

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

        template <bool TBool = std::is_integral<TItem>::value>
        void setSpaces(const std::enable_if_t<TBool, std::vector<TItem>> &num_items);
        void setSpaces(const std::vector<std::vector<TItem>> &);
        void setSpaces(const std::vector<std::shared_ptr<Space>> &spaces);

        /*!
         * @brief Transform joint item to its index in the list of all joint items.
         * @param jitem the joint item we want to get the index
         * @return the corresponding index
         */
        number getJointItemIndex(std::shared_ptr<Joint<TItem>> &jitem) const;

        /*!
         * @brief Get the corresponding joint item from its index.
         */
        TItem getJointItem(number) const;

        std::string str() const;

        virtual iterator_type begin();
        virtual iterator_type end();

        MultiDiscreteSpace &operator=(const MultiDiscreteSpace &);
        bool operator==(const MultiDiscreteSpace &other) const;
        bool operator!=(const MultiDiscreteSpace &other) const;

        /**
         * @brief Verify is the multi discrete space contains the JointItem;
         *
         * @return true
         * @return false
         */
        bool contains(const TItem &) const;

        friend std::ostream &operator<<(std::ostream &os, const MultiDiscreteSpace &sp)
        {
            os << sp.str();
            return os;
        }

    protected:
        using jitems_bimap = DiscreteSpace<TItem>::items_bimap;
        using jitems_bimap_value = jitems_bimap::value_type;

        /**
         * @brief Sets the number of joint items
         *
         */
        void setNumJItems(number);

        inline std::shared_ptr<DiscreteSpace<TItem>> cast(const std::shared_ptr<BaseSpace<TItem>> &space) const;
    };

    using MultiDiscreteStateSpace = MultiDiscreteSpace<State>;
    using MultiDiscreteActionSpace = MultiDiscreteSpace<Action>;

} // namespace sdm
