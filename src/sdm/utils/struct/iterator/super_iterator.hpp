#pragma once

#include <sdm/utils/struct/iterator.hpp>

namespace sdm
{
    namespace iterator
    {
        template <typename TItem, typename STDIterator>
        class SuperIterator : public Iterator<TItem>,
                              public std::enable_shared_from_this<SuperIterator<TItem, STDIterator>>
        {
        public:
            SuperIterator(const STDIterator &sub_iterator);
            std::shared_ptr<Iterator<TItem>> operator++();
            std::shared_ptr<Iterator<TItem>> operator+=(number n);
            std::shared_ptr<Iterator<TItem>> operator+(number n) const;
            bool operator==(const std::shared_ptr<Iterator<TItem>> &other) const;
            bool operator!=(const std::shared_ptr<Iterator<TItem>> &other) const;
            TItem &operator*();
            TItem *operator->();

        protected:
            STDIterator sub_iterator_;
        };
    } // namespace iterator

} // namespace sdm
#include <sdm/utils/struct/iterator/super_iterator.tpp>
