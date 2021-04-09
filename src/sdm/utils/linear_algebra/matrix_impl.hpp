/**
 * @file matrix_impl.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File for MatrixImpl interface.
 * @version 0.1
 * @date 12/01/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

namespace sdm
{
    /**
   * @brief The Matrix interface. To be considered as a matrix in SDM'Studio, a class must implement all the following functions. 
   * 
   * @tparam I Type of the index.
   * @tparam T Type of the values.
   */
    template <typename TCol, typename TLig, typename TValue>
    class MatrixImpl
    {
    public:
        // T operator[](const I &) const;

        // virtual T operator^(const MatrixImpl &) const = 0;
        // virtual MatrixImpl dot(const MatrixImpl &) const = 0;
    };

} // namespace sdm
