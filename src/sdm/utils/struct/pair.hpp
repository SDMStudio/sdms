#pragma once

#include <iostream>

namespace sdm
{
    template <typename T, typename U>
    using Pair = std::pair<T, U>;

    template <typename T, typename U>
    std::ostream &operator<<(std::ostream &os, const sdm::Pair<T, U> &pair)
    {
        os << "Pair{" << pair.first << ", " << pair.second << "}";
        return os;
    }
}

namespace std
{
    template <typename T, typename U>
    struct hash<sdm::Pair<T, U>>
    {
        typedef sdm::Pair<T, U> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            sdm::hash_combine(seed, in.first);
            sdm::hash_combine(seed, in.second);
            return seed;
        }
    };
}