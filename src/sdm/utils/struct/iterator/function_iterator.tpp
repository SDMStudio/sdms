#include <sdm/exception.hpp>
#include <sdm/utils/struct/iterator/combination_iterator.hpp>
#include <sdm/utils/struct/iterator/function_iterator.hpp>

namespace sdm
{
    namespace iterator
    {

        template <typename TFunction>
        FunctionIterator<TFunction>::FunctionIterator()
        {
            this->output_iterator_ = std::make_shared<CombinationIterator>();
        }

        template <typename TFunction>
        FunctionIterator<TFunction>::FunctionIterator(const std::vector<std::shared_ptr<Item>> &possible_inputs,
                                                      const std::vector<std::shared_ptr<ItemIterator>> &output_begin_iterators,
                                                      const std::vector<std::shared_ptr<ItemIterator>> &output_end_iterators)
            : possible_inputs_(possible_inputs)
        {
            assert(possible_inputs.size() == output_begin_iterators.size() && possible_inputs.size() == output_end_iterators.size());
            this->output_iterator_ = std::make_shared<CombinationIterator>(output_begin_iterators, output_end_iterators);
        }
        template <typename TFunction>
        std::shared_ptr<ItemIterator> FunctionIterator<TFunction>::copy() const
        {
            auto iter = std::make_shared<FunctionIterator<TFunction>>();
            iter->possible_inputs_ = this->possible_inputs_;
            iter->output_iterator_ = this->output_iterator_->copy();
            return iter;
        }

        template <typename TFunction>
        std::shared_ptr<ItemIterator> FunctionIterator<TFunction>::operator++()
        {
            this->output_iterator_->operator++();
            return this->shared_from_this();
        }

        template <typename TFunction>
        std::shared_ptr<ItemIterator> FunctionIterator<TFunction>::operator+=(number n)
        {
            for (number i = 0; i < n; i++)
            {
                this->operator++();
            }
            return this->shared_from_this();
        }

        template <typename TFunction>
        std::shared_ptr<ItemIterator> FunctionIterator<TFunction>::operator+(number n) const
        {
            return this->copy()->operator+=(n);
        }

        template <typename TFunction>
        bool FunctionIterator<TFunction>::operator==(const std::shared_ptr<ItemIterator> &other) const
        {
            return (this->output_iterator_->operator==(std::static_pointer_cast<FunctionIterator<TFunction>>(other)->output_iterator_));
        }

        template <typename TFunction>
        bool FunctionIterator<TFunction>::operator!=(const std::shared_ptr<ItemIterator> &other) const
        {
            return (!this->operator==(other));
        }

        template <typename TFunction>
        std::shared_ptr<Item> &FunctionIterator<TFunction>::operator*()
        {
            auto output_iter = this->output_iterator_->operator*();
            if (output_iter == nullptr)
            {
                this->temporary_item = nullptr;
            }
            else
            {
                auto outputs = std::static_pointer_cast<Joint<std::shared_ptr<Item>>>(output_iter);
                this->temporary_item = std::make_shared<TFunction>(this->possible_inputs_, *outputs);
            }
            return this->temporary_item;
        }

        template <typename TFunction>
        std::shared_ptr<Item> *FunctionIterator<TFunction>::operator->()
        {
            return &(this->operator*());
        }
    } // namespace iterator

} // namespace sdm
