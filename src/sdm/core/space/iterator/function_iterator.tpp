#include <sdm/exception.hpp>
#include <sdm/core/space/iterator/combination_iterator.hpp>
#include <sdm/core/space/iterator/function_iterator.hpp>

namespace sdm
{
    namespace iterator
    {

        template <typename TFunction, typename TBase>
        FunctionIterator<TFunction, TBase>::FunctionIterator()
        {
            this->output_iterator_ = std::make_shared<CombinationIterator<output_type>>();
        }

        template <typename TFunction, typename TBase>
        FunctionIterator<TFunction, TBase>::FunctionIterator(const std::vector<input_type> &possible_inputs,
                                                      const std::vector<std::shared_ptr<Iterator<output_type>>> &output_begin_iterators,
                                                      const std::vector<std::shared_ptr<Iterator<output_type>>> &output_end_iterators,
                                                      const std::shared_ptr<BaseSpace<output_type>> &action_space)
            : possible_inputs_(possible_inputs), action_space(action_space)
        {
            assert(possible_inputs.size() == output_begin_iterators.size() && possible_inputs.size() == output_end_iterators.size());
            this->output_iterator_ = std::make_shared<CombinationIterator<output_type>>(output_begin_iterators, output_end_iterators);
        }
        template <typename TFunction, typename TBase>
        std::shared_ptr<typename FunctionIterator<TFunction, TBase>::base_iterator_type> FunctionIterator<TFunction, TBase>::copy() const
        {
            auto iter = std::make_shared<FunctionIterator<TFunction, TBase>>();
            iter->action_space = this->action_space;
            iter->possible_inputs_ = this->possible_inputs_;
            iter->output_iterator_ = this->output_iterator_->copy();
            return iter;
        }

        template <typename TFunction, typename TBase>
        std::shared_ptr<typename FunctionIterator<TFunction, TBase>::base_iterator_type> FunctionIterator<TFunction, TBase>::operator++()
        {
            this->output_iterator_->operator++();
            return this->shared_from_this();
        }

        template <typename TFunction, typename TBase>
        std::shared_ptr<typename FunctionIterator<TFunction, TBase>::base_iterator_type> FunctionIterator<TFunction, TBase>::operator+=(number n)
        {
            for (number i = 0; i < n; i++)
            {
                this->operator++();
            }
            return this->shared_from_this();
        }

        template <typename TFunction, typename TBase>
        std::shared_ptr<typename FunctionIterator<TFunction, TBase>::base_iterator_type> FunctionIterator<TFunction, TBase>::operator+(number n) const
        {
            return this->copy()->operator+=(n);
        }

        template <typename TFunction, typename TBase>
        bool FunctionIterator<TFunction, TBase>::operator==(const std::shared_ptr<typename FunctionIterator<TFunction, TBase>::base_iterator_type> &other) const
        {
            return (this->output_iterator_->operator==(std::static_pointer_cast<FunctionIterator<TFunction, TBase>>(other)->output_iterator_));
        }

        template <typename TFunction, typename TBase>
        bool FunctionIterator<TFunction, TBase>::operator!=(const std::shared_ptr<typename FunctionIterator<TFunction, TBase>::base_iterator_type> &other) const
        {
            return (!this->operator==(other));
        }

        template <typename TFunction, typename TBase>
        std::shared_ptr<TBase> &FunctionIterator<TFunction, TBase>::getCurrent()
        {
            auto output_iter = this->output_iterator_->getCurrent();
            if (output_iter == nullptr)
            {
                this->temporary_item = nullptr;
            }
            else
            {
                auto outputs = std::static_pointer_cast<Joint<output_type>>(output_iter);
                this->temporary_item = std::make_shared<TFunction>(this->possible_inputs_, *outputs, this->action_space);
            }
            return this->temporary_item;
        }
    } // namespace iterator

} // namespace sdm
