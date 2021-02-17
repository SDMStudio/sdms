#include <sdm/utils/linear_algebra/sdms_vector.hpp>

namespace sdm
{
    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector() : TBaseVector() {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(I size) : TBaseVector(size) {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(I size, T initial_value) : TBaseVector(size, initial_value) {}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T, TBaseVector>::sdmsVector(const sdmsVector &v) : TBaseVector(v) {}

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
    T sdmsVector<I, T, TBaseVector>::at(I i) const
    {
        return TBaseVector::at(i);
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::norm_1() const
    {
        T v = 0;
        for (auto pos = this->begin(); pos != this->end(); ++pos)
        {
            v += std::abs((*this)(pos.index()));
        }
        return v;
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::norm_2() const
    {
        T v = 0;
        for (auto pos = this->begin(); pos != this->end(); ++pos)
        {
            v += std::pow((*this)(pos.index()), 2);
        }
        return v;
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
    T sdmsVector<I, T, TBaseVector>::min() const
    {
        return this->getMin().second;
    }

    template <class I, class T, class TBaseVector>
    I sdmsVector<I, T, TBaseVector>::argmin() const
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
    T sdmsVector<I, T, TBaseVector>::max() const
    {
        return this->getMax().second;
    }

    template <class I, class T, class TBaseVector>
    I sdmsVector<I, T, TBaseVector>::argmax() const
    {
        return this->getMax().first;
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::operator^(const sdmsVector &v2) const
    {
        return this->dot(v2);
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T, TBaseVector>::dot(const sdmsVector &v2) const
    {
        T product = 0;
        for (auto pos = this->begin(); pos != this->end(); ++pos)
        {
            product += (*this)(pos.index()) * v2(pos.index());
        }
        return product;
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T, TBaseVector>::operator<(const sdmsVector &v2) const
    {
        if (this->size() != v2.size())
        {
            return (this->size() < v2.size());
        }
        else
        {
            for (auto pos = this->begin(); pos != this->end(); ++pos)
            {
                if ((*this)(pos.index()) != v2(pos.index()))
                {
                    return ((*this)(pos.index()) < v2(pos.index()));
                }
            }
        }
        return false;
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

} // namespace sdm
