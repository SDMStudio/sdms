#pragma once

#include <iostream>
#include <sdm/utils/struct/vector.hpp>
#include <tuple>

namespace sdm
{
    
    /**
     * @brief sdm::Tuple is an alias to [std::tuple](https://en.cppreference.com/w/cpp/utility/tuple) class with added functionnalities. 
     * It is a generalization of sdm::Pair.
     * 
     * @tparam T... the types of the elements that the tuple stores. Empty list is supported.
     * 
     */
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