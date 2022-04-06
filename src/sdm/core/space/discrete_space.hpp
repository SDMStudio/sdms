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
     * @brief The discrete space class give a way to keep all possible values of a finite space. 
     * 
     * In order to instantiate an object of this class, you must provide the constructor method a the list of all available values.
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
        DiscreteSpace(const std::vector<T> &items)
        {
            std::vector<std::shared_ptr<Item>> titems(items.begin(), items.end());
            *this = DiscreteSpace(titems);
        }

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
        bool isStoringItems() const;
        void storeItems(bool store_items);

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

        virtual iterator_type begin();
        virtual iterator_type end();

        /**
         * @brief Get the index of an item
         */
        number getItemIndex(const std::shared_ptr<Item> &item) const;

        /**
         * @brief Get the item at a specific index
         */
        std::shared_ptr<Item> getItem(number index) const;

        /**
         * @brief Get the item at a specific index
         */
        template <typename T>
        std::shared_ptr<Item> getItemAddress(const T &item_value)
        {
            auto begin = this->begin();
            auto end = this->end();
            for (auto item = begin; item != end; item->operator++())
            {
                if (item_value == *std::static_pointer_cast<T>(*item))
                {
                    return *item;
                }
            }
            return nullptr;
        }

        /**
         * @brief Verify is the discrete space contains the std::shared_ptr<Item>;
         * 
         * @return true 
         * @return false 
         */
        bool contains(const std::shared_ptr<Item> &) const;

        int find(const std::shared_ptr<Item> &item) const;

        std::string str() const;
        std::string short_str() const;

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
        // items_bimap all_items_;


        // std::unordered_map<std::shared_ptr<Item>, number> map_item_to_int;

        /** @brief the list of possible items without their index **/
        std::vector<std::shared_ptr<Item>> list_items_;

        /**
        *  @brief Generates all joint items and maintains a bimap of indexes and corresponding pointers of joint items
        */
        void generateItems();
        bool isGenerated();

        bool store_items_ = true;
    };

} // namespace sdm