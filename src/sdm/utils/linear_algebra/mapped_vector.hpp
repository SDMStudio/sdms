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
#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/struct/vector.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/utils/struct/recursive_map.hpp>

namespace sdm
{
    /**
     * @brief Mapped vectors are vectors with specific type of indexes. They are represented by a map. 
     * 
     * @tparam TIndex the type of index
     * @tparam T the type of value (default : `double`)
     * 
     * Using map structure allows to keep only necessary values. 
     * Moreover, it allows to see vectors not only as a mapping from integer to value but also as a mapping from any type of index to values.   
     *
     */
    template <class TIndex, class T = double, class Hash = std::hash<TIndex>, class KeyEqual = std::equal_to<TIndex>>
    class MappedVector : public std::unordered_map<TIndex, T, Hash, KeyEqual>, public VectorInterface<TIndex, T>
    {
    public:
        using iterator = typename std::unordered_map<TIndex, T, Hash, KeyEqual>::iterator;
        using const_iterator = typename std::unordered_map<TIndex, T, Hash, KeyEqual>::const_iterator;

        using type = typename RecursiveMap<TIndex, T>::type;
        using value_type = typename RecursiveMap<TIndex, T>::value_type;
        using value_list_type = typename RecursiveMap<TIndex, T>::value_list_type;

        static double PRECISION;

        MappedVector();
        MappedVector(T default_value);
        MappedVector(long size, T default_value);
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
        T getValueAt(const TIndex &) const;
        void setValueAt(const TIndex &, const T &);
        void addValueAt(const TIndex &, const T &);

        bool isExist(const TIndex&) const;

        /**
         * @brief This method implements a non-commutative dot product
         * @comment: It is worth noticing that sometimes arg1.dot(arg2) !=  arg2.dot(arg1)
         * @return T 
         */
        T operator^(const MappedVector &) const;
        T operator*(const MappedVector &) const;

        template <class TOutput>
        std::shared_ptr<TOutput> add(const std::shared_ptr<TOutput> &other, double coef_this = 1., double coef_other = 1.) const
        {
            auto sum = std::make_shared<TOutput>();
            for (const auto &item : this->getIndexes())
            {
                sum->setValueAt(item, coef_this * this->getValueAt(item) + coef_other * other->getValueAt(item));
            }
            for (const auto &item : other->getIndexes())
            {
                sum->setValueAt(item, coef_this * this->getValueAt(item) + coef_other * other->getValueAt(item));
            }
            sum->finalize();
            return sum;
        }

        bool operator!=(const MappedVector &) const;
        bool operator<(const MappedVector &) const;
        bool operator==(const MappedVector &other) const;
        bool isEqual(const MappedVector &other, double precision) const;

        /**
         * @brief This method implements a non-commutative dot product
         * @comment: It is worth noticing that sometimes arg1.dot(arg2) !=  arg2.dot(arg1)
         * @return T 
         */
        T dot(const MappedVector &) const;

        T getDefault() const;
        void setDefault(double default_value);

        void setupIndexes();
        void finalize();

        std::vector<TIndex> getIndexes() const;

        static void setPrecision(double);

        std::string str() const;
        size_t size() const;

        friend std::ostream &operator<<(std::ostream &os, const MappedVector &vect)
        {
            os << vect.str();
            return os;
        }

        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);

    protected:
        T default_value_ = 0.0;
        long size_ = -1;

        std::vector<TIndex> v_indexes = {};

        bool bmin = false, bmax = false;
        std::pair<TIndex, T> pmin, pmax;

        const std::pair<TIndex, T> &getMin();
        const std::pair<TIndex, T> &getMax();
    };
} // namespace sdm
#include <sdm/utils/linear_algebra/mapped_vector.tpp>