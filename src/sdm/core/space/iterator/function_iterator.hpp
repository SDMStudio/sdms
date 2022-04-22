#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/item.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/core/space/iterator.hpp>
#include <sdm/core/space/iterator/combination_iterator.hpp>

namespace sdm
{
    namespace iterator
    {
        /**
         * @brief The function iterator is an SDMS iterator generating functions from iterable possible inputs and outputs.
         *
         * Given an list of possible inputs and a list of iterators over possible outputs, the function iterator will generate all possible functions.
         *
         * @tparam TFunction the function type. This type of function needs to accept a constructor of the form `MyFunction(const std::vector<std::shared_ptr<Item>> &inputs, const std::vector<std::shared_ptr<Item>> &outputs);`
         *
         */
        template <typename TFunction, typename TBase = TFunction>
        class FunctionIterator : public Iterator<std::shared_ptr<TBase>>,
                                 public std::enable_shared_from_this<FunctionIterator<TFunction, TBase>>
        {
        public:
            using input_type = typename TFunction::input_type;
            using output_type = typename TFunction::output_type;
            using base_iterator_type = Iterator<std::shared_ptr<TBase>>;

            FunctionIterator();

            /**
             * @brief Construct a function iterator.
             *
             * @param possible_inputs the list of possible inputs.
             * @param output_begin_iterators the output begin iterators (one by input)
             * @param output_end_iterators the output end iterators (one by input)
             */
            FunctionIterator(const std::vector<input_type> &possible_inputs,
                             const std::vector<std::shared_ptr<Iterator<output_type>>> &output_begin_iterators,
                             const std::vector<std::shared_ptr<Iterator<output_type>>> &output_end_iterators,
                             const std::shared_ptr<BaseSpace<output_type>> &action_space);

            std::shared_ptr<base_iterator_type> operator++();
            std::shared_ptr<base_iterator_type> operator+=(number n);
            std::shared_ptr<base_iterator_type> operator+(number n) const;
            bool operator==(const std::shared_ptr<base_iterator_type> &other) const;
            bool operator!=(const std::shared_ptr<base_iterator_type> &other) const;
            std::shared_ptr<TBase> &getCurrent();
            std::shared_ptr<base_iterator_type> copy() const;

        protected:
            /** @brief The output iterator */
            std::shared_ptr<base_iterator_type> output_iterator_;

            /** @brief The inputs list */
            std::vector<input_type> possible_inputs_;

            std::shared_ptr<TBase> temporary_item;

            std::shared_ptr<BaseSpace<output_type>> action_space;
        };
    } // namespace iterator

} // namespace sdm
#include <sdm/core/space/iterator/function_iterator.tpp>
