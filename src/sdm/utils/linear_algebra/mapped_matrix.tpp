#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

namespace sdm
{
    template <typename TLig, typename TCol, typename TValue>
    MappedMatrix<TLig, TCol, TValue>::MappedMatrix()
    {
    }

    template <typename TLig, typename TCol, typename TValue>
    MappedMatrix<TLig, TCol, TValue>::MappedMatrix(TValue default_value) : default_value_(default_value)
    {
    }

    template <typename TLig, typename TCol, typename TValue>
    MappedMatrix<TLig, TCol, TValue>::MappedMatrix(std::vector<long> dim, TValue default_value) : dim_(dim), default_value_(default_value)
    {
    }

    template <typename TLig, typename TCol, typename TValue>
    MappedMatrix<TLig, TCol, TValue>::MappedMatrix(const MappedMatrix &copy)
        : RecursiveMap<TLig, MappedVector<TCol, TValue>>(copy),
          dim_(copy.dim()),
          default_value_(copy.getDefault())
    {
    }

    template <typename TLig, typename TCol, typename TValue>
    MappedMatrix<TLig, TCol, TValue>::MappedMatrix(std::initializer_list<value_list_type> vals) : RecursiveMap<TLig, MappedVector<TCol, TValue>>(vals)
    {
    }

    template <typename TLig, typename TCol, typename TValue>
    TValue MappedMatrix<TLig, TCol, TValue>::getDefault() const
    {
        return this->default_value_.getDefault();
    }
    template <typename TLig, typename TCol, typename TValue>
    std::vector<long> MappedMatrix<TLig, TCol, TValue>::dim() const
    {
        return this->dim_;
    }

    template <typename TLig, typename TCol, typename TValue>
    MappedMatrix<TLig, TCol, TValue> MappedMatrix<TLig, TCol, TValue>::dot(const MappedMatrix &) const
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TLig, typename TCol, typename TValue>
    const MappedVector<TCol, TValue> &MappedMatrix<TLig, TCol, TValue>::at(const TLig &i) const
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

    template <typename TLig, typename TCol, typename TValue>
    TValue MappedMatrix<TLig, TCol, TValue>::at(const TLig &i, const TCol &j) const
    {
            return this->at(i).at(j);
    }

    template <typename TLig, typename TCol, typename TValue>
    std::string MappedMatrix<TLig, TCol, TValue>::str() const
    {
        std::ostringstream res;
        res << "<MappedMatrix>" << std::endl;

        res << "</MappedMatrix>" << std::endl;
        return res.str();
    }
} // namespace sdm
