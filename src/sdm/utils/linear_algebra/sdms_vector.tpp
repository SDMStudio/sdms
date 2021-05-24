#include <sdm/config.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
    template <class I, class T, class TBaseVector>
    double sdmsVector<I, T, TBaseVector>::PRECISION = config::PRECISION_SDMS_VECTOR;

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector() : TBaseVector() {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(sdm::size_t size) : TBaseVector(size) {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(sdm::size_t size, const T &initial_value) : TBaseVector(size, initial_value) {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(const sdmsVector &v) : TBaseVector(v), vector_indexes_(v.vector_indexes_), map_index_to_int_(v.map_index_to_int_), iterator_(v.iterator_) {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(const Vector &v) : sdmsVector(v.size())
    {
        for (size_t i = 0; i < v.size(); i++)
        {
            (*this)[i] = v[i];
        }
    }

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(const std::vector<T> &std_vector) : sdmsVector(std_vector.size())
    {
        for (int i = 0; i < std_vector.size(); i++)
        {
            (*this)[i] = std_vector[i];
        }
    }

    template <class I, class T, class TBaseVector>
    template <class AE>
    sdmsVector<I, T, TBaseVector>::sdmsVector(const boost::numeric::ublas::vector_expression<AE> &ae) : TBaseVector(ae) {}

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::sum()
    {
        T sum = 0;
        for (auto pos = TBaseVector::begin(); pos != TBaseVector::end(); ++pos)
        {
            sum += (*this)(pos.index());
        }
        return sum;
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::at(const I &index) const
    {
        assert(this->map_index_to_int_.find(index) != this->map_index_to_int_.end());
        return TBaseVector::operator[](this->map_index_to_int_.at(index));
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::getValueAt(const I &index) const
    {
        return this->at(index);
    }

    template <class I, class T, class TBaseVector>
    void sdmsVector<I, T, TBaseVector>::setValueAt(const I &index, const T &value)
    {
        if ((this->map_index_to_int_.size() < this->size()) && (this->map_index_to_int_.find(index) == this->map_index_to_int_.end()))
        {
            size_t real_index = this->map_index_to_int_.size();
            this->map_index_to_int_[index] = real_index;
            this->vector_indexes_.push_back(index);
        }
        // Init value and keep reference
        (*this)[this->map_index_to_int_.at(index)] = value;
        this->iterator_[index] = value;
    }

    template <class I, class T, class TBaseVector>
    const std::vector<I> &sdmsVector<I, T, TBaseVector>::getIndexes() const
    {
        return this->vector_indexes_;
    }

    template <class I, class T, class TBaseVector>
    void sdmsVector<I, T, TBaseVector>::setIndexes(const std::vector<I> &vector_indexes)
    {
        this->vector_indexes_ = vector_indexes;
        for (size_t i = 0; i < vector_indexes.size(); i++)
        {
            this->map_index_to_int_[vector_indexes[i]] = i;
        }
        for (auto pos = TBaseVector::begin(); pos != TBaseVector::end(); ++pos)
        {
            this->iterator_[this->vector_indexes_[pos.index()]] = (*this)(pos.index());
        }
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::norm_1() const
    {
        return boost::numeric::ublas::norm_1(*this);
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::norm_2() const
    {
        return boost::numeric::ublas::norm_2(*this);
    }

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector> sdmsVector<I, T, TBaseVector>::transpose() const
    {
        return boost::numeric::ublas::trans(*this);
    }

    template <class I, class T, class TBaseVector>
    std::pair<I, T> sdmsVector<I, T, TBaseVector>::getMin() const
    {
        T min = (*this)(0);
        I amin = 0;
        for (auto pos = TBaseVector::begin(); pos != TBaseVector::end(); ++pos)
        {
            if (min > (*this)(pos.index()))
            {
                min = (*this)(pos.index());
                amin = this->vector_indexes_[pos.index()];
            }
        }
        return {amin, min};
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::min()
    {
        return this->getMin().second;
    }

    template <class I, class T, class TBaseVector>
    I sdmsVector<I, T, TBaseVector>::argmin()
    {
        return this->getMin().first;
    }

    template <class I, class T, class TBaseVector>
    std::pair<I, T> sdmsVector<I, T, TBaseVector>::getMax() const
    {
        T max = (*this)(0);
        I amax = 0;
        for (auto pos = TBaseVector::begin(); pos != TBaseVector::end(); ++pos)
        {
            if (max < (*this)(pos.index()))
            {
                max = (*this)(pos.index());
                amax = this->vector_indexes_[pos.index()];
            }
        }
        return {amax, max};
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::max()
    {
        return this->getMax().second;
    }

    template <class I, class T, class TBaseVector>
    I sdmsVector<I, T, TBaseVector>::argmax()
    {
        return this->getMax().first;
    }

    template <class I, class T, class TBaseVector>
    template <class AE>
    T sdmsVector<I, T, TBaseVector>::dot(const boost::numeric::ublas::vector_expression<AE> &ae) const
    {
        return boost::numeric::ublas::inner_prod(*this, ae);
    }

    template <class I, class T, class TBaseVector>
    template <class AE>
    T sdmsVector<I, T, TBaseVector>::operator^(const boost::numeric::ublas::vector_expression<AE> &ae) const
    {
        return this->dot(ae);
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T, TBaseVector>::operator<=(const sdmsVector &v2) const
    {
        if (this->size() != v2.size())
        {
            return false;
        }
        else
        {
            for (auto pos = TBaseVector::begin(); pos != TBaseVector::end(); ++pos)
            {
                if (v2(pos.index()) < (*this)(pos.index()))
                {
                    return false;
                }
            }
        }
        return true;
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T, TBaseVector>::is_equal(const sdmsVector &other, double precision) const
    {
        if (this->size() != other.size())
        {
            return false;
        }
        for (auto pos = TBaseVector::begin(); pos != TBaseVector::end(); ++pos)
        {
            if (std::abs((*this)(pos.index()) - other(pos.index())) > precision)
            {
                return false;
            }
        }
        return true;
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T, TBaseVector>::operator==(const sdmsVector &other) const
    {
        return this->is_equal(other, PRECISION);
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T, TBaseVector>::operator!=(const sdmsVector &v2) const
    {
        return !this->operator==(v2);
    }

    template <class I, class T, class TBaseVector>
    std::string sdmsVector<I, T, TBaseVector>::str() const
    {
        std::ostringstream res;
        res << (*this);
        return res.str();
    }

    template <class I, class T, class TBaseVector>
    template <class Archive>
    void sdmsVector<I, T, TBaseVector>::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;
        archive &boost::serialization::base_object<TBaseVector>(*this);
        archive &make_nvp("map_index_to_int", map_index_to_int_);
        archive &make_nvp("vector_indexes", vector_indexes_);
        archive &make_nvp("iterator", iterator_);
    }

} // namespace sdm

namespace std
{
    template <class I, class T, class TBaseVector>
    struct hash<sdm::sdmsVector<I, T, TBaseVector>>
    {
        typedef sdm::sdmsVector<I, T, TBaseVector> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (const auto &pair_state_value : in)
            {
                sdm::hash_combine(seed, pair_state_value);
            }
            return seed;
        }
    };
}
