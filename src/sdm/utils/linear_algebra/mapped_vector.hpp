/**
 * @file mapped_vector.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 0.1
 * @date 07/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <unordered_map>
#include <cmath>
#include <string>
#include <iostream>
#include <assert.h>

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector_impl.hpp>

namespace sdm
{
    /**
     * @brief Mapped vectors are vectors that use map to store values of a vector. 
     * 
     * Using map structure allow to store only necessary values. Moreover, it allows to see vector in another way that is not more only a mapping from integer to value but can map any type of index to values.   
     * 
     * @tparam TIndex Type of index
     * @tparam T Type of value
     */
    template <typename TIndex, typename T = double>
    class MappedVector : public std::unordered_map<TIndex, T>, public VectorImpl<TIndex, T>
    {
    protected:
        T default_value_;
        std::size_t size_;

        std::pair<TIndex, T> getMin() const;
        std::pair<TIndex, T> getMax() const;

    public:
        MappedVector();
        MappedVector(T default_value);
        MappedVector(std::size_t size, T default_value);
        MappedVector(const MappedVector &v);

        T norm_1() const;
        T norm_2() const;

        T min() const;
        TIndex argmin() const;

        T max() const;
        TIndex argmax() const;

        T at(const TIndex &) const;
        T operator^(const MappedVector &) const;
        bool operator<(const MappedVector &) const;

        T dot(const MappedVector &v2) const;

        std::size_t size() const;

        T getDefault() const;

        std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, const MappedVector &vect)
        {
            os << vect.str();
            return os;
        }
    };
} // namespace sdm
#include <sdm/utils/linear_algebra/mapped_vector.tpp>

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::MappedVector<S, V>>
    {
        typedef sdm::MappedVector<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (auto &v : in)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v);
            }
            return seed;
        }
    };
}