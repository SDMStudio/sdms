#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/iterator.hpp>
#include <sdm/utils/struct/iterator/combination_iterator.hpp>

namespace sdm
{
    namespace iterator
    {
        /**
         * @brief The function iterator is an SDMS iterator generating functions from iterable possible inputs and outputs.
         * 
         * @tparam TFunction the function type. This type of function needs to accept a constructor of the form `MyFunction(const std::vector<std::shared_ptr<Item>> &inputs, const std::vector<std::shared_ptr<Item>> &outputs);`
         * 
         * Given an list of possible inputs and a list of iterators over possible outputs, the function iterator will generate all possible functions. 
         * 
         */
        template <typename TFunction>
        class FunctionIterator : public ItemIterator,
                                 public std::enable_shared_from_this<FunctionIterator<TFunction>>
        {
        public:
            using single_iterator_type = std::shared_ptr<ItemIterator>;

            FunctionIterator();

            /**
             * @brief Construct a function iterator.
             * 
             * @param possible_inputs the list of possible inputs.
             * @param output_begin_iterators the output begin iterators (one by input)
             * @param output_end_iterators the output end iterators (one by input)
             */
            FunctionIterator(const std::vector<std::shared_ptr<Item>> &possible_inputs,
                             const std::vector<std::shared_ptr<ItemIterator>> &output_begin_iterators,
                             const std::vector<std::shared_ptr<ItemIterator>> &output_end_iterators);

            std::shared_ptr<ItemIterator> operator++();
            std::shared_ptr<ItemIterator> operator+=(number n);
            std::shared_ptr<ItemIterator> operator+(number n) const;
            bool operator==(const std::shared_ptr<ItemIterator> &other) const;
            bool operator!=(const std::shared_ptr<ItemIterator> &other) const;
            std::shared_ptr<Item> &operator*();
            std::shared_ptr<Item> *operator->();
            std::shared_ptr<ItemIterator> copy() const;

        protected:
            /** @brief The output iterator */
            std::shared_ptr<ItemIterator> output_iterator_;

            /** @brief The inputs list */
            std::vector<std::shared_ptr<Item>> possible_inputs_;
            
            std::shared_ptr<Item> temporary_item;
        };
    } // namespace iterator

} // namespace sdm
#include <sdm/utils/struct/iterator/function_iterator.tpp>
