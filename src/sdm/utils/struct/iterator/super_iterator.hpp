#pragma once

#include <sdm/utils/struct/iterator.hpp>

namespace sdm
{
    namespace iterator
    {
        /**
         * @brief A super iterator is an SDMS iterator that simply iterate over a standard STD iterator.
         * 
         * @tparam TItem the type of value we iterate over.
         * @tparam STDIterator the type of standard iterator.
         */
        template <typename TItem, typename STDIterator>
        class SuperIterator : public Iterator<TItem>,
                              public std::enable_shared_from_this<SuperIterator<TItem, STDIterator>>
        {
        public:
            SuperIterator(const STDIterator &sub_iterator);
            std::shared_ptr<ItemIterator> copy() const;
            std::shared_ptr<Iterator<TItem>> operator++();
            std::shared_ptr<Iterator<TItem>> operator+=(number n);
            std::shared_ptr<Iterator<TItem>> operator+(number n) const;
            bool operator==(const std::shared_ptr<Iterator<TItem>> &other) const;
            bool operator!=(const std::shared_ptr<Iterator<TItem>> &other) const;
            TItem &operator*();
            TItem *operator->();

        protected:
            /** @brief the standard sub iterator. */
            STDIterator sub_iterator_;
        };
    } // namespace iterator

} // namespace sdm
#include <sdm/utils/struct/iterator/super_iterator.tpp>
