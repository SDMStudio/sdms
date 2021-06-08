#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/iterator.hpp>

namespace sdm
{
    namespace iterator
    {
        class MultiIterator : public ItemIterator,
                              public std::enable_shared_from_this<MultiIterator>
        {
        public:
            using single_iterator_type = std::shared_ptr<ItemIterator>;

            MultiIterator(){}
            MultiIterator(const std::vector<single_iterator_type> &begin_iterators,
                          const std::vector<single_iterator_type> &end_iterators);

            std::shared_ptr<ItemIterator> operator++();
            std::shared_ptr<ItemIterator> operator+=(number n);
            std::shared_ptr<ItemIterator> operator+(number n) const;
            bool operator==(const std::shared_ptr<ItemIterator> &other) const;
            bool operator!=(const std::shared_ptr<ItemIterator> &other) const;
            std::shared_ptr<Item> &operator*();
            std::shared_ptr<Item> *operator->();

        protected:
            std::vector<single_iterator_type> begin_iterators_, current_iterators_, end_iterators_;
            std::shared_ptr<Item> temporary_item;
        };
    } // namespace iterator

} // namespace sdm
