#pragma once

#include <sdm/core/space/iterator.hpp>

namespace sdm
{
    /**
     * @brief Namespace grouping all SDMS iterators.
     * 
     */
    namespace iterator
    {
        /**
         * @brief A super iterator is an SDMS iterator that simply iterate over a standard STD iterator.
         * 
         * @tparam TItem the type of value we iterate over.
         * @tparam STDIterator the type of standard iterator.
         * 
         */
        template <typename TItem, typename STDIterator>
        class SuperIterator : public Iterator<TItem>,
                              public std::enable_shared_from_this<SuperIterator<TItem, STDIterator>>
        {
        public:
            SuperIterator(const STDIterator &sub_iterator);
            std::shared_ptr<Iterator<TItem>> copy() const;
            std::shared_ptr<Iterator<TItem>> operator++();
            std::shared_ptr<Iterator<TItem>> operator+=(number n);
            std::shared_ptr<Iterator<TItem>> operator+(number n) const;
            bool operator==(const std::shared_ptr<Iterator<TItem>> &other) const;
            bool operator!=(const std::shared_ptr<Iterator<TItem>> &other) const;
            TItem &getCurrent();

        protected:
            /** @brief the standard sub iterator. */
            STDIterator sub_iterator_;
        };
    } // namespace iterator

} // namespace sdm
#include <sdm/core/space/iterator/super_iterator.tpp>
