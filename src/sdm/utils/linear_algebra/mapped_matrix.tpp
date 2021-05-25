#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/exception.hpp>

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

        // auto index_col_matrix2 = other_matrix.getIndexCol();
        // auto index_line_matrix2 = other_matrix.getIndexLine();

        // assert( index_line_matrix2 != this->index_line);
        // assert( index_col_matrix2 != this->index_col);

        // MappedMatrix<TLig, TCol, TValue> new_matrix;
        // for(const auto &line_matrix1 : this->index_line)
        // {
        //     auto map_vector = this->at(line);

        //     MappedVector<TLig, TValue> new_line;
        //     for(const auto &col_matrix2 : index_col_matrix2)
        //     {
        //         new_line[line_matrix1] = map_vector.dot(other_matrix.at(col_matrix2));
        //     }
        //     new_matrix.recursive_emplace(line_matrix1,new_line);
        // }

    }

    // template <typename TLig, typename TCol, typename TValue>
    // const std::vector<TCol> &MappedMatrix<TLig, TCol, TValue>::getIndexCol() const
    // {
    //     return this->index_col;
    // }

    // template <typename TLig, typename TCol, typename TValue>
    // const std::vector<TLig> &MappedMatrix<TLig, TCol, TValue>::getIndexLine() const
    // {
    //     return this->index_line;
    // }

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

    // template <typename TLig, typename TCol, typename TValue>
    // void &MappedMatrix<TLig, TCol, TValue>::initialize() const
    // {
    //     // Creation of index line and col 
    //     for(const auto &item_lig : *this)
    //     {
    //         this->index_line.push_back(item_lig.first);

    //         for(const auto &item_col : item_lig.second)
    //         {
    //             this->index_col.push_back(item_col.first);
    //         }
    //     }

    //     // For each Mapped Matrix TLig , TCol we associate it with a Mapped Matrix TCol, TLig 
    //     MappedMatrix<TCol,TLig, TValue> new_matrix;

    //     for(const auto &col : this->index_col)
    //     {
    //         MappedVector<TLig, TValue> new_line;
    //         for(const auto &line : this->index_line)
    //         {
    //             new_line[lig] = this->at(line,col);
    //         }
    //         new_matrix.recursive_emplace(col,new_line);
    //     }
    //     this->associated_matrix = new_matrix;
    // }

    // template <typename TLig, typename TCol, typename TValue>
    // const MappedVector<TCol, TValue> &MappedMatrix<TLig, TCol, TValue>::at(const TCol &j) const
    // {
    //     this->associated_matrix.at(j);
    // }

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
