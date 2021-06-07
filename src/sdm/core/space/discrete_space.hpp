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
#include <sdm/utils/struct/iterator/super_iterator.hpp>

/**
 * @namespace sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    /**
     * @brief The discrete space class give a way to keep all possible values of a finite space. In order to instantiate an object of this class, you must provide the constructor method, a the list of all available values.
     * 
     * @tparam std::shared_ptr<Item> The type of each element in the space. 
     *  
     */
    class DiscreteSpace : public Space
    {
    public:
        using value_type = std::shared_ptr<Item>;
        using iterator_type = Space::iterator_type;

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
        DiscreteSpace(const std::vector<std::shared_ptr<Item>> &items);

        template <typename T>
        DiscreteSpace(const std::vector<T> &items);
        /**
         * @brief Construct a new Discrete Space object from a list initializer
         */
        DiscreteSpace(std::initializer_list<std::shared_ptr<Item>> vals);

        /**
         * @brief Copy constructor
         */
        DiscreteSpace(const DiscreteSpace &copy);

        /**
         * @brief Return true because this is a discrete space
         */
        bool isDiscrete() const;

        /**
         * @brief Sample a random item from the space
         */
        std::shared_ptr<Item> sample() const;

        /**
         * @brief Get the dimension
         */
        std::vector<number> getDim() const;

        /**
         * @brief Get the number of items in the space
         */
        number getNumItems() const;

        /**
         * @brief Get all possible items in the space
         */
        std::vector<std::shared_ptr<Item>> getAll();

        virtual std::shared_ptr<iterator_type> begin();
        virtual std::shared_ptr<iterator_type> end();

        /**
         * @brief Get the index of an item
         */
        number getItemIndex(const std::shared_ptr<Item> &item) const;

        /**
         * @brief Get the item at a specific index
         */
        std::shared_ptr<Item> getItem(number index) const;

        /**
         * @brief Verify is the discrete space contains the std::shared_ptr<Item>;
         * 
         * @return true 
         * @return false 
         */
        bool contains(const std::shared_ptr<Item> &) const;

        std::string str() const;

        bool operator==(const DiscreteSpace &sp) const;
        bool operator!=(const DiscreteSpace &sp) const;

        friend std::ostream &operator<<(std::ostream &os, const DiscreteSpace &sp)
        {
            os << sp.str();
            return os;
        }

    protected:
        using items_bimap = boost::bimaps::bimap<number, std::shared_ptr<Item>>;
        using items_bimap_value = items_bimap::value_type;

        /** @brief number of possible items in the space (ex: [5, 12] --> 8 items) **/
        number num_items_;

        /** @brief the list of possible items in the space with their index **/
        items_bimap all_items_;

        /** @brief the list of possible items without their index **/
        std::vector<std::shared_ptr<Item>> list_items_;
    };

} // namespace sdm