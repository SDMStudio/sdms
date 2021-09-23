#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/iterator.hpp>

namespace sdm
{
    namespace iterator
    {
        /**
         * @brief The combination iterator provides a way to go simultaneously over multiple iterators in order to generate all combinations of items.
         * 
         * The combination iterator is a classic SDMS iterator that will efficiently generate all combination of items included in subiterators. 
         * Acceptable subiterators are all SDMS iterators. Thus, it is possible to instanciate a combination iterator over combination iterators.
         * 
         */
        class CombinationIterator : public ItemIterator,
                                    public std::enable_shared_from_this<CombinationIterator>
        {
        public:
            using single_iterator_type = std::shared_ptr<ItemIterator>;

            CombinationIterator();

            /**
             * @brief Construct a well defined combination iterator.
             * 
             * @param begin_iterators A list of begin iterators.
             * @param end_iterators A list of end iterators.
             */
            CombinationIterator(const std::vector<single_iterator_type> &begin_iterators,
                                const std::vector<single_iterator_type> &end_iterators);

            std::shared_ptr<ItemIterator> operator++();
            std::shared_ptr<ItemIterator> operator+=(number n);
            std::shared_ptr<ItemIterator> operator+(number n) const;
            bool operator==(const std::shared_ptr<ItemIterator> &other) const;
            bool operator!=(const std::shared_ptr<ItemIterator> &other) const;
            std::shared_ptr<Item> &operator*();
            std::shared_ptr<Item> *operator->();

            std::shared_ptr<ItemIterator> copy() const;

        protected:
            /** */
            std::vector<single_iterator_type> begin_iterators_, current_iterators_, end_iterators_;
            std::shared_ptr<Item> temporary_item;
            
            void debug();
        };
    } // namespace iterator

} // namespace sdm