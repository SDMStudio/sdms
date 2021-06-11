#include <sdm/utils/struct/iterator/multi_iterator.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    namespace iterator
    {

        MultiIterator::MultiIterator(const std::vector<single_iterator_type> &begin_iterators, const std::vector<single_iterator_type> &end_iterators)
            : begin_iterators_(begin_iterators),
              current_iterators_(begin_iterators),
              end_iterators_(end_iterators)
        {
            assert(begin_iterators.size() == end_iterators.size());
        }

        std::shared_ptr<ItemIterator> MultiIterator::operator++()
        {
            for (int i = this->begin_iterators_.size() - 1; i >= 0; i--)
            {
                this->current_iterators_[i] = this->current_iterators_[i]->operator+(1);
                if (*this->current_iterators_.at(i) != *this->end_iterators_.at(i))
                    break;
                else
                {

                    if (i == 0)
                    {
                        this->current_iterators_ = {};
                    }
                    else
                        this->current_iterators_[i] = this->begin_iterators_.at(i);
                }
            }
            return this->shared_from_this();
        }

        std::shared_ptr<ItemIterator> MultiIterator::operator+=(number n)
        {
            for (number i = 0; i < n; i++)
            {
                this->operator++();
            }
            return this->shared_from_this();
        }

        std::shared_ptr<ItemIterator> MultiIterator::operator+(number) const
        {
            throw exception::NotImplementedException();
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
