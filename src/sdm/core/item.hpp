#pragma once

#include <string>

namespace sdm
{
    class Item
    {
    public:
        virtual std::string str() const = 0;

        friend std::ostream &operator<<(std::ostream &os, const Item &item)
        {
            os << item.str();
            return os;
        }
    };
} // namespace sdm
