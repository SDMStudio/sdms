#include <sdm/utils/struct/iterator/multi_iterator.hpp>

namespace sdm
{
    namespace iterator
    {

        MultiIterator::MultiIterator(const std::vector<single_iterator_type> &begin_iterators, const std::vector<single_iterator_type> &end_iterators, const std::vector<single_iterator_type> &current_iterators)
            : begin_iterators_(begin_iterators),
              current_iterators_(current_iterators),
              end_iterators_(end_iterators)
        {
            assert(begin_iterators.size() == end_iterators.size());
        }

        std::shared_ptr<ItemIterator> MultiIterator::operator++()
        {
            for (int i = this->begin_iterators_.size() - 1; i >= 0; i--)
            {
                if (this->current_iterators_[i] != this->end_iterators_[i])
                {
                    this->current_iterators_.at(i)->operator++();
                    break;
                }
                else
                {
                    this->current_iterators_[i] = this->begin_iterators_.at(i);
                }
            }
            return this->shared_from_this();
        }

        bool MultiIterator::operator==(const std::shared_ptr<ItemIterator> &other) const
        {
            return (this->current_iterators_ == std::static_pointer_cast<MultiIterator>(other)->current_iterators_);
        }

        bool MultiIterator::operator!=(const std::shared_ptr<ItemIterator> &other) const
        {
            return (!this->operator==(other));
        }

        std::shared_ptr<Item> &MultiIterator::operator*()
        {
            auto joint_items = std::make_shared<Joint<std::shared_ptr<Item>>>();
            for (const auto &iter : this->current_iterators_)
            {
                joint_items->push_back(*iter);
            }
            this->temporary_item = joint_items;
            return this->temporary_item;
        }
        std::shared_ptr<Item> *MultiIterator::operator->()
        {
            return &(this->operator*());
        }
    } // namespace iterator

} // namespace sdm
