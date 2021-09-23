#pragma once

#include <sdm/parser/ast.hpp>

#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/linear_algebra/matrix_interface.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
    namespace ast
    {

        /**
         * @brief encodes the input into a mapped vector
         * 
         */
        struct vector_encoder : boost::static_visitor<std::shared_ptr<MappedVector<number, double>>>
        {

            /** @brief the size of the vector vector */
            number size_;

            vector_encoder(const number size);

            /**
             * @brief encodes a string into a mapped vector of probabilities
             * 
             * @param name a string describing the vector (i.e. "uniform") 
             * @return the corresponding mapped vector
             */
            std::shared_ptr<MappedVector<number, double>> operator()(const std::string &name) const;

            /**
             * @brief encodes a vector into a mapped vector
             * 
             * @param vector a vector of real values
             * @return the corresponding mapped vector
             */
            std::shared_ptr<MappedVector<number, double>> operator()(const std::vector<float> &vector) const;
        };

        /**
         * @brief encodes the input into a mapped matrix
         * 
         */
        struct matrix_encoder : boost::static_visitor<std::shared_ptr<MappedMatrix<number, number>>>
        {
            number rows, cols;

            matrix_encoder(number rows, number cols);

            /**
             * @brief encodes a string into a mapped matrix
             * 
             * @param str a string describing the matrix (i.e. "uniform", "identity", etc) 
             * @return the corresponding mapped matrix
             */
            std::shared_ptr<MappedMatrix<number, number>> operator()(const std::string &str) const;

            /**
             * @brief encodes a vector of vector of real values into a mapped matrix
             * 
             * @param v a vector of vector of real values
             * @return the corresponding mapped matrix 
             */
            std::shared_ptr<MappedMatrix<number, number>> operator()(const std::vector<std::vector<float>> &v) const;
        };
    } // namespace ast

} // namespace sdm
