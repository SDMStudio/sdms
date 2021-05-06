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

} // namespace sdm