#pragma once

#include <iostream>
#include <vector>

namespace sdm
{
    // template <typename T>
    // using Vector = std::vector<T>;

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const std::vector<T> &v)
    {
        os << "[";
        for (int i = 0; i < v.size(); ++i)
        {
            os << v[i];
            if (i != v.size() - 1)
                os << ", ";
        }
        os << "]";
        return os;
    }
}