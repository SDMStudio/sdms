#pragma once

#include <sdm/core/item.hpp>

namespace sdm
{
    template <typename TItem>
    class Iterator
    {
    public:
        using item_type = TItem;

        virtual ~Iterator() {}

        virtual std::shared_ptr<Iterator> operator++() = 0;
        virtual std::shared_ptr<Iterator<TItem>> operator+=(number n) = 0;
        virtual std::shared_ptr<Iterator<TItem>> operator+(number n) const = 0;

        virtual bool operator==(const std::shared_ptr<Iterator> &other) const = 0;
        virtual bool operator!=(const std::shared_ptr<Iterator> &other) const = 0;
        virtual TItem &operator*() = 0;
        virtual TItem *operator->() = 0;

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

        friend TItem &operator*(const std::shared_ptr<Iterator> &iterator)
        {
            return iterator->operator*();
        }

        // friend TItem *operator->(const std::shared_ptr<iterator> &iterator)
        // {
        //     return iterator->operator->();
        // }
    };

    using ItemIterator = Iterator<std::shared_ptr<Item>>;
} // namespace sdm
