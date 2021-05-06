/**
 * @file sdms_vector.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 0.1
 * @date 07/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <cmath>
#include <assert.h>

#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>

namespace sdm
{
    template <class TLig, class TCol, class TValue, class TBaseMatrix, class TBaseVector>
    class sdmsMatrix : public TBaseMatrix
    {
    public:
        sdmsMatrix();
        sdmsMatrix(std::size_t n_lig, std::size_t n_col);
    };

    /**
     * @brief Sparse matrix are matrix that store only non-zero values.
     * 
     * @tparam TLig Type of lines
     * @tparam TCol Type of columns
     */
    template <typename TLig, class TCol, class TValue>
    using SparseMatrix = sdmsMatrix<TLig, TCol, TValue, boost::numeric::ublas::mapped_matrix<TValue>, boost::numeric::ublas::mapped_vector<TValue>>;

    /**
     * @brief Dense matrix are plain matrix.
     * 
     * @tparam TLig Type of lines
     * @tparam TCol Type of columns
     */
    template <typename TLig, class TCol, class TValue>
    using DenseMatrix = sdmsMatrix<TLig, TCol, TValue, boost::numeric::ublas::matrix<TValue>, boost::numeric::ublas::vector<TValue>>;

} // namespace sdm

#include <sdm/utils/linear_algebra/sdms_matrix.tpp>
