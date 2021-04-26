
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/struct/pair.hpp>

namespace sdm
{
    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector() : RecursiveMap<TIndex, T>(), default_value_(0.), size_(-1)
    {
    }

    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector(T default_value) : RecursiveMap<TIndex, T>(), default_value_(default_value), size_(-1)
    {
    }

    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector(long size, T default_value) : RecursiveMap<TIndex, T>(), default_value_(default_value), size_(size)
    {
    }

    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::MappedVector(const MappedVector &v) : RecursiveMap<TIndex, T>(v), default_value_(v.getDefault()), size_(v.size())
    {
    }

    template <typename TIndex, typename T>
    MappedVector<TIndex, T>::~MappedVector() {}

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::norm_1() const
    {
        T v = 0;
        for (const auto &item : *this)
        {
            v += std::abs(item.second);
        }
        return v;
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::norm_2() const
    {
        T v = 0;
        for (const auto &item : *this)
        {
            v += std::pow(item.second, 2);
        }
        return v;
    }

    template <typename TIndex, typename T>
    std::pair<TIndex, T> MappedVector<TIndex, T>::getMin() const
    {
        T min = std::numeric_limits<T>::max();
        TIndex amin;
        for (const auto &item : *this)
        {
            if (min > item.second)
            {
                amin = item.first;
                min = item.second;
            }
        }
        if (min == std::numeric_limits<T>::max())
        {
            amin = TIndex();
            min = this->default_value_;
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
        T max = -std::numeric_limits<T>::max();
        TIndex amax;
        for (const auto &item : *this)
        {
            if (max < item.second)
            {
                amax = item.first;
                max = item.second;
            }
        }
        if (max == -std::numeric_limits<T>::max())
        {
            amax = TIndex();
            max = this->default_value_;
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

    template <typename TIndex, typename T>
    bool MappedVector<TIndex, T>::operator==(const MappedVector<TIndex, T> &v2) const
    {
        double eps = 0.0000001;
        if (std::abs(this->getDefault() - v2.getDefault()) > eps)
        {
            return false;
        }
        
        for (const auto &v : *this)
        {
            if (v2.find(v.first) == v2.end())
            {
                return false;
            }
            else
            {
                if (std::abs(v2.at(v.first) - v.second) > eps)
                {
                    return false;
                }
            }
        }

        for (const auto &v : v2)
        {
            if (this->find(v.first) == this->end())
            {
                return false;
            }
            else
            {
                if (std::abs(this->at(v.first) - v.second) > eps)
                {
                    return false;
                }
            }
        }
        return true;
    }

    template <typename TIndex, typename T>
    bool MappedVector<TIndex, T>::operator!=(const MappedVector<TIndex, T> &v2) const
    {
        return (!this->operator==(v2));
    }

    template <typename TIndex, typename T>
    T MappedVector<TIndex, T>::at(const TIndex &i) const
    {
        if (this->find(i) != this->end())
        {
            return RecursiveMap<TIndex, T>::at(i);
        }
        else
        {
            return this->default_value_;
        }
    }

    template <typename TIndex, typename T>
    bool MappedVector<TIndex, T>::operator<(const MappedVector &v2) const
    {
        for (const auto &item : *this)
        {
            if (v2.find(item.first) != v2.end())
            {
                return item.second < v2.at(item.first);
            }
        }
        for (const auto &item : v2)
        {
            if (this->find(item.first) != this->end())
            {
                return this->at(item.first) < item.second;
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
        for (const auto &item : *this)
        {
            product += item.second * v2.at(item.first);
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
    std::vector<TIndex> MappedVector<TIndex, T>::getIndexes() const
    {
        std::vector<TIndex> v_indexes;
        for (const auto &p_i_v : *this)
        {
            v_indexes.push_back(p_i_v.first);
        }
        return v_indexes;
    }

    template <typename TIndex, typename T>
    std::string MappedVector<TIndex, T>::str() const
    {
        std::ostringstream res;
        std::string size = ((this->size_ > 0) ? std::to_string(this->size_) : "?");
        res << "[" << size << "]";
        res << "(";
        for (const auto &val : *this)
        {
            res << val.first << " : " << val.second << ", ";
        }
        res << "default"
            << " : " << this->default_value_;
        res << ")";
        return res.str();
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::MappedVector<S, V>>
    {
        typedef sdm::MappedVector<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            std::map<S, V> ordered(in.begin(), in.end());
            for (const auto &v : ordered)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v);
            }
            return seed;
        }
    };
}