#pragma once

#include <sdm/core/item.hpp>

namespace sdm
{
    /**
     * @brief Common interface to all SDMS Iterators.
     *
     * Iterators in SDMS are different to std iterators. The usage is similar but SDMS iterators can be seen as super iterators. Indeed, they usually are kind of iterator over iterator.
     * The advantage of this hierarchy in SDMS iterators is that we can use super iterators over other super iterators and without care about the type of sub-iterators.
     *
     * @tparam TItem the type of value we iterate over.
     *
     */
    template <typename TItem>
    class Iterator
    {
    public:
        using item_type = TItem;

        virtual ~Iterator() {}

        /**
         * @brief Copy function.
         *
         * @return the copied iterator
         */
        virtual std::shared_ptr<Iterator<TItem>> copy() const = 0;

        /**
         * @brief Increment the iterator.
         *
         * @return the incremented iterator
         */
        virtual std::shared_ptr<Iterator> operator++() = 0;

        virtual std::shared_ptr<Iterator> next()
        {
            return this->operator++();
        }

        /**
         * @brief Increment the iterator of n steps.
         *
         * @param n the number of step
         * @return the n-step incremented iterator
         */
        virtual std::shared_ptr<Iterator<TItem>> operator+=(number n) = 0;

        /**
         * @brief Get the n-step incremented iterator without modifying the current iterator.
         *
         * @param n the number of step
         * @return the n-step incremented copy iterator
         */
        virtual std::shared_ptr<Iterator<TItem>> operator+(number n) const = 0;

        /**
         * @brief Check if two iterators are equals.
         * This function is used to check the end.
         */
        virtual bool operator==(const std::shared_ptr<Iterator> &other) const = 0;

        /**
         * @brief Check if two iterators are equals.
         * This function is used to check the end.
         */
        virtual bool equal(const std::shared_ptr<Iterator> &other) const
        {
            return this->operator==(other);
        }

        /**
         * @brief Check if two iterators are differents.
         */
        virtual bool operator!=(const std::shared_ptr<Iterator> &other) const = 0;

        /**
         * @brief Get a reference to the item.
         *
         * @return TItem& the item
         */
        virtual TItem &getCurrent() = 0;

        friend TItem &operator*(const std::shared_ptr<Iterator<TItem>> &iterator)
        {
            return iterator->getCurrent();
        }

        friend bool operator==(const std::shared_ptr<Iterator> &iterator1, const std::shared_ptr<Iterator> &iterator2)
        {
            return iterator1->operator==(iterator2);
        }

        friend bool operator!=(const std::shared_ptr<Iterator> &iterator1, const std::shared_ptr<Iterator> &iterator2)
        {
            return iterator1->operator!=(iterator2);
        }

        friend std::shared_ptr<Iterator> operator++(const std::shared_ptr<Iterator> &iterator)
        {
            return iterator->operator++();
        }
    };

    using ItemIterator = Iterator<std::shared_ptr<Item>>;
} // namespace sdm
