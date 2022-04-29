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
#include <sdm/utils/linear_algebra/matrix_interface.hpp>
// #include <tuple>

namespace sdm
{
    /**
     * @brief Mapped matrices are matrices that use map to store values. Can be view as a sparse matrix with templated indexes.
     *
     * @tparam TLig the type of index for the lines
     * @tparam TCol the type of index for the colums
     * @tparam TValue the type of value (default : `double`)
     */
    template <typename TLig, typename TCol, typename TValue = double>
    class MappedMatrix : public RecursiveMap<TLig, MappedVector<TCol, TValue>>, public MatrixInterface<TLig, TCol, TValue>
    {
    protected:
        std::vector<long> dim_ = {};
        MappedVector<TCol, TValue> default_value_;

    public:
        using type = typename RecursiveMap<TLig, MappedVector<TCol, TValue>>::type;
        using value_type = typename RecursiveMap<TLig, MappedVector<TCol, TValue>>::value_type;
        using value_list_type = typename RecursiveMap<TLig, MappedVector<TCol, TValue>>::value_list_type;
        using vector_type = MappedVector<TCol, TValue>;

        MappedMatrix();
        MappedMatrix(TValue default_value);
        MappedMatrix(std::vector<long> dim, TValue default_value);
        MappedMatrix(const MappedMatrix &copy);
        MappedMatrix(std::initializer_list<value_list_type> vals);

        TValue getDefault() const;
        TValue getValueAt(const TLig &, const TCol &) const;
        void setValueAt(const TLig &, const TCol &, const TValue &);
        void setValuesAt(const TLig &lig, const MappedVector<TCol, TValue> &vector);

        std::vector<long> dim() const;
        MappedMatrix dot(const MappedMatrix &) const;
        const MappedVector<TCol, TValue> &at(const TLig &i) const;
        TValue at(const TLig &i, const TCol &j) const;

        std::string str() const;

        std::vector<Pair<TLig, TCol>> getIndexes();
        std::vector<std::tuple<TLig, TCol, TValue>> getAllElement();

        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &archive, const unsigned int);
    };
} // namespace sdm
#include <sdm/utils/linear_algebra/mapped_matrix.tpp>

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
                // Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v);
            }
        }
    };
}