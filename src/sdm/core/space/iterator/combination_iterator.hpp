#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/item.hpp>
#include <sdm/core/space/iterator.hpp>

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
        template <typename TItem, typename TBuild = Joint<TItem>>
        class CombinationIterator : public Iterator<TItem>,
                                    public std::enable_shared_from_this<CombinationIterator<TItem>>
        {
        public:
            using base_iterator_type = Iterator<TItem>;
            using single_iterator_type = std::shared_ptr<Iterator<TItem>>;

            CombinationIterator();

            /**
             * @brief Construct a well defined combination iterator.
             * 
             * @param begin_iterators A list of begin iterators.
             * @param end_iterators A list of end iterators.
             */
            CombinationIterator(const std::vector<single_iterator_type> &begin_iterators,
                                const std::vector<single_iterator_type> &end_iterators);

            std::shared_ptr<base_iterator_type> operator++();
            std::shared_ptr<base_iterator_type> operator+=(number n);
            std::shared_ptr<base_iterator_type> operator+(number n) const;
            bool operator==(const std::shared_ptr<base_iterator_type> &other) const;
            bool operator!=(const std::shared_ptr<base_iterator_type> &other) const;
            TItem &getCurrent();

            std::shared_ptr<base_iterator_type> copy() const;

            /** @brief The list of iterators to be combined. */
            std::vector<single_iterator_type> begin_iterators_, current_iterators_, end_iterators_;
        protected:
            TItem temporary_item;
            
            void debug();
        };
    } // namespace iterator

} // namespace sdm

#include <sdm/core/space/iterator/combination_iterator.tpp>