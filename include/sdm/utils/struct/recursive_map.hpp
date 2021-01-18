/**
 * @file recursive_map.cpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 0.1
 * @date 17/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <map>
#include <vector>

// To be modified to change the map representation to store data.
template <typename K, typename V>
using map_t = std::map<K, V>;

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

    /**
     * @class RecursiveMap
     * @brief The recursive map class (i.e. map<T0, map<T1, ..... , map<TN-1, TN>)
     * 
     * @tparam T0 the type of the first map item
     * @tparam T1 the type of the second map item
     * @tparam Ts the type of the other map items
     */
    template <class T0, class T1, class... Ts>
    class RecursiveMap;

    template <class T0, class T1, class... Ts>
    using RecursiveMap_t = typename RecursiveMap<T0, T1, Ts...>::type;

    template <class T0, class T1, class... Ts>
    using value_t = typename RecursiveMap<T0, T1, Ts...>::value_type;

    /**
     * @brief RecursiveMap specialization when it is simple map.
     * 
     * @tparam T0 the type of the first map item
     * @tparam T1 the type of the second map item
     */
    template <class T0, class T1>
    class RecursiveMap<T0, T1> : public map_t<T0, T1>
    {
    public:
        using type = RecursiveMap<T0, T1>;
        using value_type = std::pair<const T0, T1>;

        RecursiveMap() : map_t<T0, T1>() {}
        RecursiveMap(const map_t<T0, T1> &x) : map_t<T0, T1>(x) {}
        RecursiveMap(std::initializer_list<value_type> vals) : map_t<T0, T1>(vals) {}

        /**
         * @brief Emplace a specific element.
         * 
         * @param v0 the key
         * @param v1 the value
         */
        void recursive_emplace(T0 v0, T1 v1);

        /**
         * @brief Get a specific item
         */
        T1 operator()(T0 v0);

        friend std::ostream &operator<<(std::ostream &os, RecursiveMap<T0, T1> rmap)
        {
            os << "{ ";
            for (auto it = rmap.begin(); it != rmap.end(); ++it)
            {
                os << it->first << " : " << it->second;
                os << ((std::next(it) != rmap.end()) ? ", " : "");
            }
            os << " }";
            return os;
        }
    };

    template <class T0, class T1, class T2, class... Ts>
    class RecursiveMap<T0, T1, T2, Ts...> : public map_t<T0, RecursiveMap_t<T1, T2, Ts...>>
    {
    public:
        using type = RecursiveMap<T0, RecursiveMap_t<T1, T2, Ts...>>;
        using value_type = std::pair<const T0, value_t<T1, T2, Ts...>>;
        using value_list_type = std::pair<const T0, std::initializer_list<value_t<T1, T2, Ts...>>>;

        RecursiveMap() : map_t<T0, RecursiveMap_t<T1, T2, Ts...>>() {}
        RecursiveMap(const map_t<T0, RecursiveMap_t<T1, T2, Ts...>> &x) : map_t<T0, RecursiveMap_t<T1, T2, Ts...>>(x) {}
        RecursiveMap(std::initializer_list<value_list_type> vals) : map_t<T0, RecursiveMap_t<T1, T2, Ts...>>()
        {
            for (const auto &v : vals)
            {
                RecursiveMap_t<T1, T2, Ts...> tmp(v.second);
                this->emplace(v.first, tmp);
            }
        }

        /**
         * @brief Recursively emplace a specific element.
         * 
         * @param v0 first key
         * @param v1 second key
         * @param v2 third key
         * @param vs other keys
         */
        void recursive_emplace(T0 v0, T1 v1, T2 v2, Ts... vs);

        /**
         * @brief Get a specific item
         */
        RecursiveMap<T1, T2, Ts...> operator()(T0 v0);

        /**
         * @brief Get a specific item
         */
        auto operator()(T0 v0, T1 v1, Ts... vs);

        friend std::ostream &operator<<(std::ostream &os, RecursiveMap<T0, T1, T2, Ts...> rmap)
        {
            os << "{ ";
            for (auto it = rmap.begin(); it != rmap.end(); ++it)
            {
                os << it->first << " : " << it->second;
                os << ((std::next(it) != rmap.end()) ? ", " : "");
            }
            os << " }";
            return os;
        }
    };

} // namespace sdm

#include <sdm/utils/struct/recursive_map.tpp>