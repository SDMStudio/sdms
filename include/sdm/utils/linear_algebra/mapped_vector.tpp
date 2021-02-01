
#include <sdm/utils/linear_algebra/mapped_vector.hpp>

namespace sdm
{
    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector() : std::map<TIndex, T>(), size_(0), default_value_(0)
    {
    }

    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector(T default_value) : std::map<TIndex, T>(), default_value_(default_value)
    {
    }

    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector(TIndex size, T default_value) : std::map<TIndex, T>(), default_value_(default_value), size_(size)
    {
    }

    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector(const MappedVector &v) : std::map<TIndex, T>(v), default_value_(v.getDefault()), size_(v.size())
    {
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::norm_1() const
    {
        T v = 0;
        for (auto item : *this)
        {
            v += std::abs(item.second);
        }
        return v;
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::norm_2() const
    {
        T v = 0;
        for (auto item : *this)
        {
            v += std::pow(item.second, 2);
        }
        return v;
    }

    template <typename TIndex, typename T>
    std::pair<TIndex, T> MappedVector<TIndex, T>::getMin() const
    {
        T min = this->default_value_;
        TIndex amin;
        for (auto item : *this)
        {
            if (min > item.second)
            {
                amin = item.first;
                min = item.second;
            }
        }
        return {amin, min};
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::min() const
    {
        return this->getMin().second;
    }

    template <typename TIndex, typename T>
    TIndex MappedVector<TIndex, T>::argmin() const
    {
        return this->getMin().first;
    }

    template <typename TIndex, typename T>
    std::pair<TIndex, T> MappedVector<TIndex, T>::getMax() const
    {
        T max = this->default_value_;
        TIndex amax;
        for (auto item : *this)
        {
            if (max < item.second)
            {
                amax = item.first;
                max = item.second;
            }
        }
        return {amax, max};
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::max() const
    {
        return this->getMax().second;
    }

    template <typename TIndex, typename T>
    TIndex MappedVector<TIndex, T>::argmax() const
    {
        return this->getMax().first;
    }

    // template <typename TIndex, typename T>
    // T MappedVector<TIndex, T>::operator+(const MappedVector &v2) const
    // {
    // }

    // template <typename TIndex, typename T>
    // T MappedVector<TIndex, T>::operator-(const MappedVector &v2) const
    // {
    // }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::at(TIndex i)
    {
        if (this->find(i) != this->end())
        {
            return (*this)[i];
        }
        else
        {
            return this->default_value_;
        }
    }

    template <typename TIndex, typename T>
    bool MappedVector<TIndex, T>::operator<(const MappedVector &v2)
    {
        for (auto item : *this)
        {
            if (v2.find(item.first) != v2.end())
            {
                return item.second < v2.at(item.first);
            }
        }
        for (auto item : v2)
        {
            if (this->find(item.first) != this->end())
            {
                return  this->at(item.first) < item.second;
            }
        }
        return false;
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::operator^(const MappedVector &v2) const
    {
        return this->dot(v2);
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::dot(const MappedVector &v2) const
    {
        T product = 0;
        for (auto item : *this)
        {
            if (v2.find(item.first) != v2.end())
            {
                product += item.second * v2.at(item.first);
            }
        }
        return product;
    }

    template <typename TIndex, typename T>
    std::size_t MappedVector<TIndex, T>::size() const
    {
        return this->size_;
    }


    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::getDefault() const
    {
        return this->default_value_;
    }

    template <typename TIndex, typename T>
    std::string MappedVector<TIndex, T>::str() const
    {
        std::ostringstream res;
        std::string size = ((this->size_ > 0) ? std::to_string(this->size_) : "?");
        res << "[" << size << "]";
        res << "(";
        for (auto val : *this)
        {
            res << val.first << " : " << val.second << ", ";
        }
        res << "default"
            << " : " << this->default_value_;
        res << ")";
        return res.str();
    }

} // namespace sdm