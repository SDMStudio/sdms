/**
 * @file discrete_space.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File for DiscreteSpace class.
 * @version 1.0
 * @date 28/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>
#include <boost/bimap.hpp>

#include <sdm/types.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/space.hpp>

/**
 * @namespace sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    /**
     * @brief The discrete space class give a way to keep all possible values of a finite space. In order to instantiate an object of this class, you must provide the constructor method, a the list of all available values.
     * 
     * @tparam TItem The type of each element in the space. 
     *  
     */
    template <typename TItem>
    class DiscreteSpaceBase : public Space
    {
    protected:
        typedef boost::bimaps::bimap<number, TItem> items_bimap;
        typedef typename items_bimap::value_type items_bimap_value;

        /** @brief number of possible items in the space (ex: [5, 12] --> 8 items) **/
        number num_items_;

        /** @brief the list of possible items in the space with their index **/
        items_bimap all_items_;

    public:
        using value_type = TItem;

        /**
         * @brief Construct a new Discrete Space object (default)
         * 
         */
        DiscreteSpaceBase();

        /**
         * @brief Construct a new Discrete Space object
         * 
         * @param items a list of possible items in the space
         */
        DiscreteSpaceBase(const std::vector<TItem> &items);

        /**
         * @brief Copy constructor
         */
        DiscreteSpaceBase(const DiscreteSpaceBase<TItem> &copy);

        /**
         * @brief Construct a new Discrete Space object from a list initializer
         */
        DiscreteSpaceBase(std::initializer_list<TItem> vals);

        /**
         * @brief Return true because this is a discrete space
         */
        bool isDiscrete() const;

        /**
         * @brief Sample a random item from the space
         */
        TItem sample() const;

        /**
         * @brief Get the dimension
         */
        std::vector<number> getDim() const;

        /**
         * @brief Get the Nummber of Items in the space
         */
        number getNumItems() const;

        /**
         * @brief Get all items in the space
         */
        std::vector<TItem> getAll();

        /**
         * @brief Get the index of an item
         */
        number getItemIndex(const TItem &item) const;

        /**
         * @brief Get the item at a specific index
         */
        TItem getItem(number index) const;

        std::string str() const;

        DiscreteSpaceBase &operator=(const DiscreteSpaceBase &sp);

        bool operator==(const DiscreteSpaceBase &sp) const;
        bool operator!=(const DiscreteSpaceBase &sp) const;

        friend std::ostream &operator<<(std::ostream &os, const DiscreteSpaceBase &sp)
        {
            os << sp.str();
            return os;
        }
    };

    class DiscreteSpace : public DiscreteSpaceBase<number>
    {
    public:
        DiscreteSpace();

        /**
         * @brief Construct a new Discrete Space object
         * 
         * @param number_item the number of items in the space. If k then {0, 1, ..., k-1} are possible items.
         */
        DiscreteSpace(int number_item);

        /**
         * @brief Construct a new Discrete Space object
         * 
         * @param items a list of possible items in the space
         */
        DiscreteSpace(const std::vector<number> &items);

        /**
         * @brief Copy constructors
         */
        DiscreteSpace(const DiscreteSpace &copy);

        /**
         * @brief Construct a new Discrete Space object from a list initializer
         */
        DiscreteSpace(std::initializer_list<number> vals);

    };

    using StringDiscreteSpace = DiscreteSpaceBase<std::string>;

} // namespace sdm

#include <sdm/core/space/discrete_space.tpp>
