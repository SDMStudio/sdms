#include <sdm/config.hpp>
#include <sdm/utils/linear_algebra/sdms_vector.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
    template <class I, class T, class TBaseVector>
    double sdmsVector<I, T,TBaseVector>::PRECISION = config::PRECISION_SDMS_VECTOR;

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T,TBaseVector>::sdmsVector(){}

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T,TBaseVector>::sdmsVector(std::shared_ptr<std::unordered_map<I, size_t>> map_element_to_index, std::shared_ptr<std::unordered_map<I, T>> map_element_to_value, double default_value) :
        map_element_to_index_(map_element_to_index)
    {        
        TBaseVector vector(map_element_to_index_->size(),default_value);
        for(const auto &element_to_index : *map_element_to_index)
        {
            vector[element_to_index.second] = map_element_to_value->at(element_to_index.first);
            vector_element_.push_back(element_to_index.first);
        }
        this->tbasevector_ = vector;
    }

    template <class I, class T, class TBaseVector>
    sdmsVector<I, T,TBaseVector>::sdmsVector(std::vector<I> vector_element, std::vector<T> vector_value, double default_value)
    {        
        assert(vector_element.size() == vector_value.size());
        
        std::unordered_map<I, size_t> element_to_index;

        TBaseVector vector(vector_element.size(),default_value);
        for(size_t index = 0; index <vector_element.size(); index ++)
        {
            vector[index] = vector_value[index];

            vector_element_.push_back(vector_element[index]);
            element_to_index[vector_element[index]] = index;
        }
        this->tbasevector_ = vector;
        this->map_element_to_index_ = std::make_shared<std::unordered_map<I, size_t>>(element_to_index);
    }


    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::sum() const
    {
        T sum = 0.0;
        for (auto pos = this->tbasevector_.begin(); pos != this->tbasevector_.end(); ++pos)
        {
            sum += this->tbasevector_(pos.index());
        }
        return sum;
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::at(const I &element) const
    {
        assert(this->map_element_to_index_->find(element) != this->map_element_to_index_->end());
        return this->tbasevector_(this->map_element_to_index_->at(element));
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::getValueAt(const I &index) const
    {
        return this->at(index);
    }

    template <class I, class T, class TBaseVector>
    void sdmsVector<I, T,TBaseVector>::setValueAt(const I &element, const T &value)
    {
        assert(this->map_element_to_index_->find(element) != this->map_element_to_index_->end());
        this->tbasevector_[this->map_element_to_index_->at(element)] = value;
    }

    template <class I, class T, class TBaseVector>
    void sdmsVector<I, T,TBaseVector>::addValueAt(const I &element, const T &value)
    {
        assert(this->map_element_to_index_->find(element) != this->map_element_to_index_->end());
        this->tbasevector_[this->map_element_to_index_->at(element)] = this->getValueAt(element) + value;
    }

    template <class I, class T, class TBaseVector>
    const std::vector<I> &sdmsVector<I, T,TBaseVector>::getIndexes() const
    {
        return this->vector_element_;
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::norm_1() const
    {
        return boost::numeric::ublas::norm_1(this->tbasevector_);
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::norm_2() const
    {
        return boost::numeric::ublas::norm_2(this->tbasevector_);
    }

    template <class I, class T, class TBaseVector>
    std::pair<I, T> sdmsVector<I, T,TBaseVector>::getMin() const
    {
        T min = this->tbasevector_(0);
        I amin = 0.0;
        for (auto pos = this->tbasevector_.begin(); pos != this->tbasevector_.end(); ++pos)
        {
            if (min > this->tbasevector_(pos.index()))
            {
                min = this->tbasevector_(pos.index());
                amin = this->vector_element_[pos.index()];
            }
        }
        return {amin, min};
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::min()
    {
        return this->getMin().second;
    }

    template <class I, class T, class TBaseVector>
    I sdmsVector<I, T,TBaseVector>::argmin()
    {
        return this->getMin().first;
    }

    template <class I, class T, class TBaseVector>
    std::pair<I, T> sdmsVector<I, T,TBaseVector>::getMax() const
    {
        T max = this->tbasevector_(0);
        I amax = 0.0;
        for (auto pos = this->tbasevector_.begin(); pos != this->tbasevector_.end(); ++pos)
        {
            if (max < this->tbasevector_(pos.index()))
            {
                max = this->tbasevector_(pos.index());
                amax = this->vector_element_[pos.index()];
            }
        }
        return {amax, max};
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::max()
    {
        return this->getMax().second;
    }

    template <class I, class T, class TBaseVector>
    I sdmsVector<I, T,TBaseVector>::argmax()
    {
        return this->getMax().first;
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::dot(const sdmsVector &v2) const
    {
        return boost::numeric::ublas::inner_prod(this->tbasevector_, v2.tbasevector_);
    }

    template <class I, class T, class TBaseVector>
    T sdmsVector<I, T,TBaseVector>::operator^(const sdmsVector &v2) const
    {
        return this->dot(v2);
    }

    template <class I, class T, class TBaseVector>
    size_t sdmsVector<I, T,TBaseVector>::size() const
    {
        return this->map_element_to_index_->size();
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T,TBaseVector>::operator<=(const sdmsVector &v2) const
    {
        assert(this->getIndexes() == v2.getIndexes());

        for (const auto &element : this->getIndexes())
        {
            if (v2.getValueAt(element) < this->getValueAt(element))
            {
                return false;
            }
        }
        return true;
    }
    template <class I, class T, class TBaseVector>
    std::shared_ptr<std::unordered_map<I, size_t>> sdmsVector<I, T,TBaseVector>::getMapElementToIndex() const
    {
        return this->map_element_to_index_;
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T,TBaseVector>::is_equal(const sdmsVector &other, double precision) const
    {
        if( this->getIndexes() != other.getIndexes())
        {
            return false;
        }

        for (const auto &element : this->getIndexes())
        {
            if (std::abs(this->getValueAt(element) - other.getValueAt(element)) > precision)
            {
                return false;
            }
        }
        return true;
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T,TBaseVector>::operator==(const sdmsVector &other) const
    {
        return this->is_equal(other, PRECISION);
    }

    template <class I, class T, class TBaseVector>
    bool sdmsVector<I, T,TBaseVector>::operator!=(const sdmsVector &v2) const
    {
        return !this->operator==(v2);
    }

    template <class I, class T, class TBaseVector>
    std::string sdmsVector<I, T,TBaseVector>::str() const
    {
        std::ostringstream res;
        res << tbasevector_;
        return res.str();
    }

    template <class I, class T, class TBaseVector>
    template <class Archive>
    void sdmsVector<I, T,TBaseVector>::serialize(Archive &archive, const unsigned int)
    {
        using boost::serialization::make_nvp;
        archive &boost::serialization::base_object<TBaseVector>(this->tbasevector_);
        archive &make_nvp("map_index_to_int", map_element_to_index_);
        archive &make_nvp("vector_indexes", vector_element_);
    }

} // namespace sdm

namespace std
{
    // template <class I, class T, class TBaseVector>
    // struct hash<sdm::sdmsVector<I, T,TBaseVector>>
    // {
    //     typedef sdm::sdmsVector<I, T,TBaseVector> argument_type;
    //     typedef std::size_t result_type;
    //     inline result_type operator()(const argument_type &in) const
    //     {
    //         size_t seed = 0;
    //         for (const auto &pair_state_value : in)
    //         {
    //             sdm::hash_combine(seed, pair_state_value);
    //         }
    //         return seed;
    //     }
    // };
}
