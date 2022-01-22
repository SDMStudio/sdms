#include <random>

#include <sdm/types.hpp>
#include <sdm/common.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/distribution.hpp>

namespace sdm
{
    // template <typename T>
    // std::shared_ptr<DiscreteDistribution<T>> Distribution<T>::toDiscreteDistribution()
    // {
    //     return std::static_pointer_cast<DiscreteDiscreteDistribution<T>>(this->shared_from_this());
    // }

    template <typename T>
    DiscreteDistribution<T>::DiscreteDistribution()
    {
    }

    template <typename T>
    DiscreteDistribution<T>::DiscreteDistribution(const DiscreteDistribution<T> &copy) : probabilities(copy.probabilities)
    {
    }

    // template <typename T>
    // T DiscreteDistribution<T>::sample() const
    // {
    //     std::discrete_distribution<size_t> distrib(this->probabilities_.begin(), this->probabilities_.end());
    //     auto n = distrib(common::global_urng());

    //     return this->bimap_item_to_index_.right.at(n);
    // }

    // template <typename T>
    // double DiscreteDistribution<T>::getProbability(const T &item, const T &) const
    // {
    //     const auto &iterator = this->bimap_item_to_index_.left.find(item);
    //     return (iterator == this->bimap_item_to_index_.left.end()) ? 0 : this->probabilities_.at(iterator->second);
    // }

    // template <typename T>
    // void DiscreteDistribution<T>::setProbability(const T &item, double proba)
    // {
    //     // assert((proba >= -0.) && (proba <= 1.1));
    //     if (proba > 0)
    //     {
    //         if (this->bimap_item_to_index_.left.find(item) == this->bimap_item_to_index_.left.end())
    //         {
    //             auto idx = this->probabilities_.size();
    //             this->bimap_item_to_index_.insert(bimap_pair(item, idx));
    //             this->probabilities_.push_back(proba);
    //         }
    //         else
    //         {
    //             this->probabilities_[this->bimap_item_to_index_.left.at(item)] = proba;
    //         }
    //     }
    //     else
    //     {
    //         if (this->bimap_item_to_index_.left.find(item) != this->bimap_item_to_index_.left.end())
    //         {
    //             this->probabilities_[this->bimap_item_to_index_.left.at(item)] = 0.;
    //         }
    //     }
    // }

    template <typename T>
    T DiscreteDistribution<T>::sample() const
    {
        // Get a random number between 0 and 1
        double epsilon = std::rand() / (double(RAND_MAX)), cumul;

        for (auto pair_item_proba : this->probabilities)
        {
            cumul += pair_item_proba.second;
            if (epsilon < cumul)
            {
                return pair_item_proba.first;
                break;
            }
        }
        throw sdm::exception::Exception("Incomplete DiscreteDistribution");
    }

    template <typename T>
    double DiscreteDistribution<T>::getProbability(const T &item, const T &) const
    {
        const auto &iterator = this->probabilities.find(item);
        return (iterator == probabilities.end()) ? 0 : iterator->second;
    }

    template <typename T>
    void DiscreteDistribution<T>::setProbability(const T &item, double proba)
    {
        // assert((proba >= -0.) && (proba <= 1.1));
        if (proba > 0)
        {
            this->probabilities[item] = proba;
        }
        else
        {
            const auto &iterator = this->probabilities.find(item);
            if (iterator != probabilities.end())
                this->probabilities.erase(iterator);
        }
    }

} // namespace sdm