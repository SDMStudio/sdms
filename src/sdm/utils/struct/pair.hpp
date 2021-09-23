#pragma once

#include <iostream>
#include <sdm/utils/struct/vector.hpp>

namespace sdm
{
    /**
     * @brief sdm::Pair is an alias to [std::pair](https://en.cppreference.com/w/cpp/utility/pair) class with added functionnalities.
     * 
     * @tparam T the type of the first element that the pair stores.
     * @tparam U the type of the second element that the pair stores.
     * 
     */
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