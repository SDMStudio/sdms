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
    class DiscreteSpace : public Space
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
        DiscreteSpace();

        /**
         * @brief Construct a new Discrete Space object
         * 
         * @param items a list of possible items in the space
         */
        DiscreteSpace(const std::vector<TItem> &items);

        /**
         * @brief Copy constructor
         */
        DiscreteSpace(const DiscreteSpace<TItem> &copy);

        /**
         * @brief Construct a new Discrete Space object from a list initializer
         */
        DiscreteSpace(std::initializer_list<TItem> vals);

        /**
         * @brief Construct a new Discrete Space Base object
         */
        template <bool TBool = std::is_integral<TItem>::value>
        DiscreteSpace(std::enable_if_t<TBool, int> num_items);

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

        DiscreteSpace &operator=(const DiscreteSpace &sp);

        bool operator==(const DiscreteSpace &sp) const;
        bool operator!=(const DiscreteSpace &sp) const;

        friend std::ostream &operator<<(std::ostream &os, const DiscreteSpace &sp)
        {
            os << sp.str();
            return os;
        }
    };

} // namespace sdm

#include <sdm/core/space/discrete_space.tpp>
