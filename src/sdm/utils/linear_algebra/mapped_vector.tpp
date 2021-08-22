
#include <sdm/config.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/utils/struct/pair.hpp>

namespace sdm
{
    template <class TIndex, class T, class Hash, class KeyEqual>
    double MappedVector<TIndex, T, Hash, KeyEqual>::PRECISION = config::PRECISION_MAPPED_VECTOR;

    template <class TIndex, class T, class Hash, class KeyEqual>
    MappedVector<TIndex, T, Hash, KeyEqual>::MappedVector() : std::unordered_map<TIndex, T, Hash, KeyEqual>(), default_value_(0.), size_(-1)
    {
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    MappedVector<TIndex, T, Hash, KeyEqual>::MappedVector(T default_value) : std::unordered_map<TIndex, T, Hash, KeyEqual>(), default_value_(default_value), size_(-1)
    {
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    MappedVector<TIndex, T, Hash, KeyEqual>::MappedVector(long size, T default_value) : std::unordered_map<TIndex, T, Hash, KeyEqual>(), default_value_(default_value), size_(size)
    {
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    MappedVector<TIndex, T, Hash, KeyEqual>::MappedVector(const MappedVector &v)
        : std::unordered_map<TIndex, T, Hash, KeyEqual>(v),
          default_value_(v.default_value_),
          size_(v.size_),
          v_indexes(v.v_indexes),
          bmin(v.bmin),
          bmax(v.bmax),
          pmin(v.pmin),
          pmax(v.pmax)
    {
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    MappedVector<TIndex, T, Hash, KeyEqual>::MappedVector(std::initializer_list<value_list_type> vals) : std::unordered_map<TIndex, T, Hash, KeyEqual>(vals), default_value_(0), size_(-1)
    {
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    MappedVector<TIndex, T, Hash, KeyEqual>::~MappedVector() {}

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::norm_1() const
    {
        T v = 0.0;
        for (const auto &item : *this)
        {
            v += std::abs(item.second);
        }
        return v;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::at(const TIndex &i) const
    {
        if (this->find(i) != this->end())
        {
            return std::unordered_map<TIndex, T, Hash, KeyEqual>::at(i);
        }
        else
        {
            return this->default_value_;
        }
    }
    template <class TIndex, class T, class Hash, class KeyEqual>
    bool MappedVector<TIndex, T, Hash, KeyEqual>::isExist(const TIndex&i) const
    {
        auto iterator = this->find(i);
        return (iterator == this->end() ) ? false : true;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::getValueAt(const TIndex &index) const
    {
        return this->at(index);
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    void MappedVector<TIndex, T, Hash, KeyEqual>::setValueAt(const TIndex &index, const T &value)
    {
        (*this)[index] = value;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    void MappedVector<TIndex, T, Hash, KeyEqual>::addValueAt(const TIndex &index, const T &value)
    {
        (*this)[index] = this->getValueAt(index) + value;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::norm_2() const
    {
        T v = 0.0;
        for (const auto &item : *this)
        {
            v += std::pow(item.second, 2);
        }
        return v;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    const std::pair<TIndex, T> &MappedVector<TIndex, T, Hash, KeyEqual>::getMin()
    {
        if (!this->bmin)
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

            this->pmin = {amin, min};
        }

        return this->pmin;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::min()
    {
        return this->getMin().second;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    TIndex MappedVector<TIndex, T, Hash, KeyEqual>::argmin()
    {
        return this->getMin().first;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    const std::pair<TIndex, T> &MappedVector<TIndex, T, Hash, KeyEqual>::getMax()
    {
        if (!this->bmax)
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

            this->pmax = {amax, max};
        }

        return this->pmax;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::max()
    {
        return this->getMax().second;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    TIndex MappedVector<TIndex, T, Hash, KeyEqual>::argmax()
    {
        return this->getMax().first;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    void MappedVector<TIndex, T, Hash, KeyEqual>::setPrecision(double precision)
    {
        MappedVector<TIndex, T, Hash, KeyEqual>::PRECISION = precision;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    bool MappedVector<TIndex, T, Hash, KeyEqual>::is_equal(const MappedVector<TIndex, T, Hash, KeyEqual> &other, double precision) const
    {
        if (this->size() != other.size())
        {
            return false;
        }
        if (std::abs(this->getDefault() - other.getDefault()) > precision)
        {
            return false;
        }

        for (const auto &pair_state_value : *this)
        {
            if (other.find(pair_state_value.first) == other.end())
            {
                return false;
            }
            else
            {
                if (std::abs(other.at(pair_state_value.first) - pair_state_value.second) > precision)
                {
                    return false;
                }
            }
        }

        return true;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    bool MappedVector<TIndex, T, Hash, KeyEqual>::operator==(const MappedVector<TIndex, T, Hash, KeyEqual> &other) const
    {
        return this->is_equal(other, PRECISION);
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    bool MappedVector<TIndex, T, Hash, KeyEqual>::operator!=(const MappedVector<TIndex, T, Hash, KeyEqual> &v2) const
    {
        return (!this->operator==(v2));
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    bool MappedVector<TIndex, T, Hash, KeyEqual>::operator<(const MappedVector &v2) const
    {
        for (const auto &item : *this)
        {
            if (item.second > v2.at(item.first))
            {
                return false;
            }
        }
        for (const auto &item : v2)
        {
            if (item.second < this->at(item.first))
            {
                return false;
            }
        }
        return true;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::operator*(const MappedVector &v2) const
    {
        return this->dot(v2);
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::operator^(const MappedVector &v2) const
    {
        return this->dot(v2);
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::dot(const MappedVector &v2) const
    {
        T product = 0.0;
        for (const auto &item : *this)
        {
            product += item.second * v2.at(item.first);
        }

        return product;
    }

    // template <class TOutput>
    // template <class TIndex, class T, class Hash, class KeyEqual>
    // std::shared_ptr<TOutput> MappedVector<TIndex, T, Hash, KeyEqual>::add(const std::shared_ptr<TOutput> &v2) const
    // {
    // }

    template <class TIndex, class T, class Hash, class KeyEqual>
    size_t MappedVector<TIndex, T, Hash, KeyEqual>::size() const
    {
        return std::unordered_map<TIndex, T, Hash, KeyEqual>::size();
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    T MappedVector<TIndex, T, Hash, KeyEqual>::getDefault() const
    {
        return this->default_value_;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    void MappedVector<TIndex, T, Hash, KeyEqual>::setDefault(double default_value)
    {
        this->default_value_ = default_value;
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    void MappedVector<TIndex, T, Hash, KeyEqual>::setupIndexes()
    {
        this->v_indexes.clear();
        if (this->v_indexes.size() == 0)
        {
            for (const auto &p_i_v : *this)
            {
                this->v_indexes.push_back(p_i_v.first);
            }
        }
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    std::vector<TIndex> MappedVector<TIndex, T, Hash, KeyEqual>::getIndexes() const
    {
        if (this->v_indexes.size() > 0)
        {
            assert((this->v_indexes.size() == this->size()));
            return this->v_indexes;
        }
        else
        {
            std::vector<TIndex> indexes = {};
            for (const auto &p_i_v : *this)
            {
                indexes.push_back(p_i_v.first);
            }
            return indexes;
        }
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    void MappedVector<TIndex, T, Hash, KeyEqual>::finalize()
    {
        this->setupIndexes();
    }

    template <class TIndex, class T, class Hash, class KeyEqual>
    std::string MappedVector<TIndex, T, Hash, KeyEqual>::str() const
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

    template <class TIndex, class T, class Hash, class KeyEqual>
    template <class Archive>
    void MappedVector<TIndex, T, Hash, KeyEqual>::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;

        archive &boost::serialization::base_object<std::unordered_map<TIndex, T, Hash, KeyEqual>>(*this);
        archive &make_nvp("default_value", default_value_);
        archive &make_nvp("size", size_);
        archive &make_nvp("PRECISION", PRECISION);

        archive &make_nvp("bmin", bmin);
        archive &make_nvp("bmax", bmax);

        if (v_indexes.size() > 0)
            archive &make_nvp("vector_indexes", v_indexes);

        if (bmin)
        {
            archive &make_nvp("pair_min", pmin);
        }
        if (bmax)
        {
            archive &make_nvp("pair_max", pmax);
        }
    }

} // namespace sdm

namespace std
{
    template <class Key, class Value, class Hash, class KeyEqual>
    struct hash<sdm::MappedVector<Key, Value, Hash, KeyEqual>>
    {
        typedef sdm::MappedVector<Key, Value, Hash, KeyEqual> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return this->operator()(in, sdm::MappedVector<Key, Value, Hash, KeyEqual>::PRECISION);
        }

        inline result_type operator()(const argument_type &in, double precision) const
        {
            size_t seed = 0;
            double inverse_of_precision = 1. / precision;
            std::map<Key, Value> ordered(in.begin(), in.end());
            std::map<Key, int> rounded;
            for (const auto &pair_item_value : in)
            {
                rounded.emplace(pair_item_value.first, lround(inverse_of_precision * pair_item_value.second));
            }
            for (const auto &v : rounded)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, v);
            }
            return seed;
        }
    };
}