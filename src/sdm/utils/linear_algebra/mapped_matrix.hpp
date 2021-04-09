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

#include <sdm/types.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/matrix_impl.hpp>

namespace sdm
{
    /**
     * @brief Mapped tensors are tensors that use map to store values. 
     * 
     * @tparam TIndex the type of indexes 
     */
    template <typename TLig, typename TCol, typename TValue = double>
    class MappedMatrix : public RecursiveMap<TLig, MappedVector<TCol, TValue>>, public MatrixImpl<TLig, TCol, TValue>
    {
    protected:
        std::vector<long> dim_ = {};
        TValue default_value_;

    public:
        using type = typename RecursiveMap<TLig, MappedVector<TCol, TValue>>::type;
        using value_type = typename RecursiveMap<TLig, MappedVector<TCol, TValue>>::value_type;
        using value_list_type = typename RecursiveMap<TLig, MappedVector<TCol, TValue>>::value_list_type;

        MappedMatrix() {}
        MappedMatrix(TValue default_value) : default_value_(default_value) {}
        MappedMatrix(std::vector<long> dim, TValue default_value) : dim_(dim), default_value_(default_value) {}
        MappedMatrix(const MappedMatrix &copy) : RecursiveMap<TLig, MappedVector<TCol, TValue>>(copy), dim_(copy.dim()), default_value_(copy.getDefault()) {}
        MappedMatrix(std::initializer_list<value_list_type> vals) : RecursiveMap<TLig, MappedVector<TCol, TValue>>(vals) {}

        TValue getDefault() const
        {
            return this->default_value_;
        }
        std::vector<long> dim() const
        {
            return this->dim_;
        }

        MappedMatrix dot(const MappedMatrix &) const
        {
            throw sdm::exception::NotImplementedException();
        }

        MappedVector<TCol, TValue> at(const TLig &i) const
        {
            if (this->find(i) != this->end())
            {
                return RecursiveMap<TLig, MappedVector<TCol, TValue>>::at(i);
            }
            else
            {
                return this->default_value_;
            }
        }

        std::string str() const
        {
            std::ostringstream res;
            res << "<MappedMatrix>" << std::endl;

            res << "</MappedMatrix>" << std::endl;
            return res.str();
        }
    };
} // namespace sdm

namespace std
{
    template <typename TLig, typename TCol, typename TValue>
    struct hash<sdm::MappedMatrix<TLig, TCol, TValue>>
    {
        typedef sdm::MappedMatrix<TLig, TCol, TValue> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (const auto &v : in)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v);
            }
        }
    };
}