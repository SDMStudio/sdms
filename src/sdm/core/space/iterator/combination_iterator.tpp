#include <sdm/core/space/iterator/combination_iterator.hpp>
#include <sdm/exception.hpp>

namespace sdm
{
    namespace iterator
    {
        template <typename TItem, typename TBuild>
        CombinationIterator<TItem, TBuild>::CombinationIterator() {}

        template <typename TItem, typename TBuild>
        CombinationIterator<TItem, TBuild>::CombinationIterator(const std::vector<single_iterator_type> &begin_iterators, const std::vector<single_iterator_type> &end_iterators)
            : begin_iterators_(begin_iterators),
              current_iterators_(begin_iterators),
              end_iterators_(end_iterators)
        {
            assert(begin_iterators.size() == end_iterators.size());
        }

        template <typename TItem, typename TBuild>
        std::shared_ptr<typename CombinationIterator<TItem, TBuild>::base_iterator_type> CombinationIterator<TItem, TBuild>::operator++()
        {
            // From the last to the first sub iterator
            for (int i = this->begin_iterators_.size() - 1; i >= 0; i--)
            {
                // Increment i-th sub iterator
                this->current_iterators_[i] = this->current_iterators_[i]->operator+(1);

                // Compare incremented i-th iterator to end iterator,
                if (this->current_iterators_.at(i)->operator!=(this->end_iterators_.at(i)))
                {
                    // If the incremented i-th sub iterator is different to end iterator, stop the for loop and return incremented iterator
                    break;
                }
                else
                {
                    // If the incremented i-th iterator is equal to the end iterator
                    if (i == 0)
                    {
                        // If the incremented i-th sub iterator is the 0-th (first) end iterator, return global end iterator
                        this->current_iterators_ = {};
                    }
                    else
                    {
                        // If the incremented i-th sub iterator is not first iterator, assign i-th begin iterator to i-th iterator and continue
                        this->current_iterators_[i] = this->begin_iterators_.at(i);
                    }
                }
            }
            return this->shared_from_this();
        }

        template <typename TItem, typename TBuild>
        std::shared_ptr<typename CombinationIterator<TItem, TBuild>::base_iterator_type> CombinationIterator<TItem, TBuild>::operator+=(number n)
        {
            for (number i = 0; i < n; i++)
            {
                this->operator++();
            }
            return this->shared_from_this();
        }

        template <typename TItem, typename TBuild>
        std::shared_ptr<typename CombinationIterator<TItem, TBuild>::base_iterator_type> CombinationIterator<TItem, TBuild>::operator+(number n) const
        {
            return this->copy()->operator+=(n);
        }

        template <typename TItem, typename TBuild>
        std::shared_ptr<typename CombinationIterator<TItem, TBuild>::base_iterator_type> CombinationIterator<TItem, TBuild>::copy() const
        {
            // Create a copy iterator that keep same pointer on begin and end iterators but copy current iterators
            auto multi_it = std::make_shared<CombinationIterator<TItem>>();
            multi_it->begin_iterators_ = this->begin_iterators_;
            multi_it->end_iterators_ = this->end_iterators_;
            for (const auto &iter : this->current_iterators_)
            {
                multi_it->current_iterators_.push_back(iter->copy());
            }
            return multi_it;
        }

        template <typename TItem, typename TBuild>
        bool CombinationIterator<TItem, TBuild>::operator==(const std::shared_ptr<typename CombinationIterator<TItem, TBuild>::base_iterator_type> &other) const
        {
            return (this->current_iterators_ == std::static_pointer_cast<CombinationIterator<TItem>>(other)->current_iterators_);
        }
        
        template <typename TItem, typename TBuild>
        bool CombinationIterator<TItem, TBuild>::operator!=(const std::shared_ptr<typename CombinationIterator<TItem, TBuild>::base_iterator_type> &other) const
        {
            return (!this->operator==(other));
        }

        template <typename TItem, typename TBuild>
        TItem &CombinationIterator<TItem, TBuild>::getCurrent()
        {
            if (this->current_iterators_.empty())
            {
                this->temporary_item = nullptr;
            }
            else
            {
                std::vector<TItem> list_items;
                std::transform(current_iterators_.begin(), current_iterators_.end(), std::back_inserter(list_items), [](const single_iterator_type &iter)
                               { return iter->getCurrent(); });
                this->temporary_item = std::make_shared<TBuild>(list_items);
            }
            return this->temporary_item;
        }

        template <typename TItem, typename TBuild>
        void CombinationIterator<TItem, TBuild>::debug()
        {
            std::cout << "<current-iteraor>\n";
            for (const auto &iter : this->current_iterators_)
            {
                std::cout << "\t<iterator address=" << iter << " *iter=" << *iter << " *iter->str=" << (*iter)->str() << " />" << std::endl;
            }
            std::cout << "<current-iterator/>" << std::endl;
        }
    } // namespace iterator

} // namespace sdm
