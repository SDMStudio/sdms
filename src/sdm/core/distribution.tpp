#include <random>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/core/distribution.hpp>

namespace sdm
{
    template <typename T>
    T DiscreteDistribution<T>::sample() const
    {
        // std::vector<double> list_probabilities = tools::extractValues(this->probabilities_);
        std::discrete_distribution<size_t> distrib(this->probabilities_.begin(), this->probabilities_.end());
        return this->bimap_item_to_index_.right.at(distrib(common::global_urng()));
    }

    template <typename T>
    double DiscreteDistribution<T>::getProbability(const T &item, const T &) const
    {
        const auto &iterator = this->bimap_item_to_index_.left.find(item);
        return (iterator == this->bimap_item_to_index_.left.end()) ? 0 : this->probabilities_.at(iterator->second);
    }

    template <typename T>
    void DiscreteDistribution<T>::setProbability(const T &item, double proba)
    {
        assert((proba >= 0) && (proba <= 1));
        if (proba > 0)
        {
            if (this->bimap_item_to_index_.left.find(item) == this->bimap_item_to_index_.left.end())
            {
                this->bimap_item_to_index_.insert(bimap_pair(item, this->probabilities_.size()));
                this->probabilities_.push_back(proba);
            }
            else
            {
                this->probabilities_[this->bimap_item_to_index_.left.at(item)] = proba;
            }
        }
    }
} // namespace sdm