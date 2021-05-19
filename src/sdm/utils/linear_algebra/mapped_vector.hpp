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

#include <iostream>
#include <unordered_map>
#include <cmath>
#include <string>
#include <vector>
#include <assert.h>

#include <boost/serialization/base_object.hpp>

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/struct/recursive_map.hpp>

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
    class MappedVector : public RecursiveMap<TIndex, T>, public VectorImpl<TIndex, T>
    {
    public:
        using iterator = typename std::unordered_map<TIndex, T>::iterator;
        using const_iterator = typename std::unordered_map<TIndex, T>::const_iterator;

        using type = typename RecursiveMap<TIndex, T>::type;
        using value_type = typename RecursiveMap<TIndex, T>::value_type;
        using value_list_type = typename RecursiveMap<TIndex, T>::value_list_type;

        MappedVector();
        MappedVector(T);
        MappedVector(long, T);
        MappedVector(const MappedVector &);
        MappedVector(std::initializer_list<value_list_type>);
        virtual ~MappedVector();

        T norm_1() const;
        T norm_2() const;

        T min();
        TIndex argmin();

        T max();
        TIndex argmax();

        T at(const TIndex &) const;

        /**
         * @brief This method implements a non-commutative dot product
         * @comment: It is worth noticing that sometimes arg1.dot(arg2) !=  arg2.dot(arg1)
         * @return T 
         */
        T operator^(const MappedVector &) const;
        T operator*(const MappedVector &) const;

        bool operator==(const MappedVector &) const;
        bool operator!=(const MappedVector &) const;
        bool operator<(const MappedVector &) const;

        /**
         * @brief This method implements a non-commutative dot product
         * @comment: It is worth noticing that sometimes arg1.dot(arg2) !=  arg2.dot(arg1)
         * @return T 
         */
        T dot(const MappedVector &) const;

        T getDefault() const;

        void setIndexes();

        std::vector<TIndex> getIndexes() const;

        void setPrecision(double);

        std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, const MappedVector &vect)
        {
            os << vect.str();
            return os;
        }

    protected:
        T default_value_;
        long size_ = -1;
        double precision = 0.00001;

        std::vector<TIndex> v_indexes = {};

        bool bmin = false, bmax = false;
        std::pair<TIndex, T> pmin, pmax;

        const std::pair<TIndex, T> &getMin();
        const std::pair<TIndex, T> &getMax();

        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);
    };
} // namespace sdm
#include <sdm/utils/linear_algebra/mapped_vector.tpp>