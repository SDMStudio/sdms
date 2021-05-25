#include <sdm/utils/linear_algebra/sdms_vector.hpp>

namespace sdm
{
    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector() : TBaseVector() {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(sdm::size_t size) : TBaseVector(size) {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(sdm::size_t size, const T &initial_value) : TBaseVector(size, initial_value) {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(const sdmsVector &v) : TBaseVector(v) {}

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
        for (auto pos = this->begin(); pos != this->end(); ++pos)
        {
            sum += (*this)(pos.index());
        }
        return sum;
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::at(const I &i) const
    {
        return TBaseVector::operator[](i);
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
        for (auto pos = this->begin(); pos != this->end(); ++pos)
        {
            if (min > (*this)(pos.index()))
            {
                min = (*this)(pos.index());
                amin = pos.index();
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
        for (auto pos = this->begin(); pos != this->end(); ++pos)
        {
            if (max < (*this)(pos.index()))
            {
                max = (*this)(pos.index());
                amax = pos.index();
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
    T sdmsVector<I, T, TBaseVector>::operator^(const boost::numeric::ublas::vector_expression<AE> &ae)
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
            for (auto pos = this->begin(); pos != this->end(); ++pos)
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
    bool sdmsVector<I, T, TBaseVector>::operator==(const sdmsVector &v2) const
    {
        if (this->size() != v2.size())
        {
            return false;
        }
        for (auto pos = this->begin(); pos != this->end(); ++pos)
        {
            if ((*this)(pos.index()) != v2(pos.index()))
            {
                return false;
            }
        }

        for (auto pos = v2.begin(); pos != v2.end(); ++pos)
        {
            if ((*this)(pos.index()) != v2(pos.index()))
            {
                return false;
            }
        }

        return true;
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
        // archive &make_nvp("map_item_to_index", map_item_to_index_);
    }

} // namespace sdm
