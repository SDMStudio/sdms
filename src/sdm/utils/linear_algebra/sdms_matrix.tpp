#include <sdm/utils/linear_algebra/sdms_matrix.hpp>

namespace sdm
{
    template <class TLig, class TCol, class TValue, class TBaseMatrix, class TBaseVector>
    sdmsMatrix<TLig, TCol, TValue, TBaseMatrix, TBaseVector>::sdmsMatrix()
    {
    }

    template <class TLig, class TCol, class TValue, class TBaseMatrix, class TBaseVector>
    sdmsMatrix<TLig, TCol, TValue, TBaseMatrix, TBaseVector>::sdmsMatrix(std::size_t n_lig, std::size_t n_col) : TBaseMatrix(n_lig, n_col)
    {
    }

    template <class TLig, class TCol, class TValue, class TBaseMatrix, class TBaseVector>
    sdmsMatrix<TLig, TCol, TValue, TBaseMatrix, TBaseVector>::sdmsMatrix(const sdmsMatrix &m) : TBaseMatrix(m) {}

    template <class TLig, class TCol, class TValue, class TBaseMatrix, class TBaseVector>
    template <class AE>
    sdmsMatrix<TLig, TCol, TValue, TBaseMatrix, TBaseVector>::sdmsMatrix(const boost::numeric::ublas::matrix_expression<AE> &ae) : TBaseMatrix(ae) {}

    template <class TLig, class TCol, class TValue, class TBaseMatrix, class TBaseVector>
    sdmsMatrix<TLig, TCol, TValue, TBaseMatrix, TBaseVector> sdmsMatrix<TLig, TCol, TValue, TBaseMatrix, TBaseVector>::transpose() const
    {
        return boost::numeric::ublas::trans(*this);
    }

    template <class TLig, class TCol, class TValue, class TBaseMatrix, class TBaseVector>
    TBaseVector sdmsMatrix<TLig, TCol, TValue, TBaseMatrix, TBaseVector>::operator^(const TBaseVector &vector) const
    {
        return TBaseVector(boost::numeric::ublas::prod(*this, vector));
    }

} // namespace sdm