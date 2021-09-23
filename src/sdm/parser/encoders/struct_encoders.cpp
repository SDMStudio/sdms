#include <sdm/parser/encoders/struct_encoders.hpp>

namespace sdm
{
    namespace ast
    {

        vector_encoder::vector_encoder(const number size) : boost::static_visitor<std::shared_ptr<MappedVector<number, double>>>()
        {
            this->size_ = size;
        }

        std::shared_ptr<MappedVector<number, double>> vector_encoder::operator()(const std::string &name) const
        {
            std::shared_ptr<MappedVector<number, double>> v = std::make_shared<MappedVector<number>>(size_, 0);
            if (name == "uniform")
            {
                for (number i = 0; i < this->size_; ++i)
                {
                    v->setValueAt(i, 1.0 / this->size_);
                }
            }

            return v;
        }

        std::shared_ptr<MappedVector<number, double>> vector_encoder::operator()(const std::vector<float> &vector) const
        {
            std::shared_ptr<MappedVector<number, double>> v = std::make_shared<MappedVector<number>>(size_, 0);
            for (number i = 0; i < vector.size(); ++i)
            {
                v->setValueAt(i, vector[i]);
            }

            return v;
        }

        matrix_encoder::matrix_encoder(number rows, number cols) : rows(rows), cols(cols) {}

        //! \param str the way to encode the matrix
        //! \brief encodes a string into a matrix
        std::shared_ptr<MappedMatrix<number, number>> matrix_encoder::operator()(const std::string &str) const
        {
            number s, s_;
            std::shared_ptr<MappedMatrix<number, number>> matrix = std::make_shared<MappedMatrix<number, number>>(0.);

            if (str == "uniform")
            {
                for (s = 0; s < rows; ++s)
                {
                    for (s_ = 0; s_ < cols; ++s_)
                    {
                        matrix->setValueAt(s, s_, 1.0 / cols);
                    }
                }
            }

            else if (str == "identity")
            {
                for (s = 0; s < rows; ++s)
                {
                    for (s_ = 0; s_ < cols; ++s_)
                    {
                        matrix->setValueAt(s, s_, (s == s_) ? 1.0 : 0.);
                    }
                }
            }
            return matrix;
        }

        //! \param v the matrix
        //! \brief encodes a matrix into a matrix
        std::shared_ptr<MappedMatrix<number, number>> matrix_encoder::operator()(const std::vector<std::vector<float>> &v) const
        {
            number s, s_;
            std::shared_ptr<MappedMatrix<number, number>> matrix = std::make_shared<MappedMatrix<number, number>>(0.);

            for (s = 0; s < rows; ++s)
            {
                for (s_ = 0; s_ < cols; ++s_)
                {
                    if (v[s][s_] != 0.)
                    {
                        matrix->setValueAt(s, s_, v[s][s_]);
                    }
                }
            }

            return matrix;
        }
    } // namespace ast

} // namespace sdm
