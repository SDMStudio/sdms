#pragma once

#include <iostream>
#include <sdm/utils/struct/vector.hpp>
#include <tuple>

namespace sdm
{
    template <class... T>
    using Tuple = std::tuple<T...>;

    template <class... T>
    std::ostream &operator<<(std::ostream &os, sdm::Tuple<T...> const &tuple)
    {
        os << "Tuple{";
        std::apply(
            [&os](T const &...tupleArgs) {
                std::size_t n{0};
                ((os << tupleArgs << (++n != sizeof...(T) ? ", " : "")), ...);
            },
            tuple);
        os << "}";
        return os;
    }
}

namespace std
{
    namespace
    {

        // Code from boost
        // Reciprocal of the golden ratio helps spread entropy
        //     and handles duplicates.
        // See Mike Seymour in magic-numbers-in-boosthash-combine:
        //     http://stackoverflow.com/questions/4948780

        template <class T>
        inline void hash_combine(std::size_t &seed, T const &v)
        {
            seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        // Recursive template code derived from Matthieu M.
        template <class Tuple, size_t Index = std::tuple_size<Tuple>::value - 1>
        struct HashValueImpl
        {
            static void apply(size_t &seed, Tuple const &tuple)
            {
                HashValueImpl<Tuple, Index - 1>::apply(seed, tuple);
                sdm::hash_combine(seed, std::get<Index>(tuple));
            }
        };

        template <class Tuple>
        struct HashValueImpl<Tuple, 0>
        {
            static void apply(size_t &seed, Tuple const &tuple)
            {
                sdm::hash_combine(seed, std::get<0>(tuple));
            }
        };
    }

    template <class... TT>
    struct hash<sdm::Tuple<TT...>>
    {
        size_t
        operator()(sdm::Tuple<TT...> const &tt) const
        {
            size_t seed = 0;
            HashValueImpl<sdm::Tuple<TT...>>::apply(seed, tt);
            return seed;
        }
    };
}