#include <sdm/utils/struct/iterator/super_iterator.hpp>

namespace sdm
{
    namespace iterator
    {

        template <typename TItem, typename STDIterator>
        SuperIterator<TItem, STDIterator>::SuperIterator(const STDIterator &sub_iterator) : sub_iterator_(sub_iterator)
        {
        }

        template <typename TItem, typename STDIterator>
        std::shared_ptr<Iterator<TItem>> SuperIterator<TItem, STDIterator>::operator++()
        {
            this->sub_iterator_++;
            return this->shared_from_this();
        }

        template <typename TItem, typename STDIterator>
        std::shared_ptr<Iterator<TItem>> SuperIterator<TItem, STDIterator>::operator+=(number n)
        {
            this->sub_iterator_+=n;
            return this->shared_from_this();
        }

        template <typename TItem, typename STDIterator>
        std::shared_ptr<Iterator<TItem>> SuperIterator<TItem, STDIterator>::operator+(number n) const
        {
            return std::make_shared<SuperIterator>(this->sub_iterator_ + n);
        }

        template <typename TItem, typename STDIterator>
        bool SuperIterator<TItem, STDIterator>::operator==(const std::shared_ptr<Iterator<TItem>> &other) const
        {
            return (this->sub_iterator_ == std::static_pointer_cast<SuperIterator>(other)->sub_iterator_);
        }

        template <typename TItem, typename STDIterator>
        bool SuperIterator<TItem, STDIterator>::operator!=(const std::shared_ptr<Iterator<TItem>> &other) const
        {
            return (!this->operator==(other));
        }

        template <typename TItem, typename STDIterator>
        TItem &SuperIterator<TItem, STDIterator>::operator*()
        {
            return *this->sub_iterator_;
        }
        template <typename TItem, typename STDIterator>
        TItem *SuperIterator<TItem, STDIterator>::operator->()
        {
            return &(*this->sub_iterator_);
        }
    } // namespace iterator

} // namespace sdm
