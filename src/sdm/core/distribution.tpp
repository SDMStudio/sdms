#include <sdm/types.hpp>
#include <sdm/core/distribution.hpp>

namespace sdm
{
    template <typename T>
    T DiscreteDistribution<T>::sample() const
    {
        std::vector<double> list_probabilities = tools::extractValues(this->probabilities_);
        std::discrete_distribution ditrib(list_probabilities);
        return distrib(common::global_urng());
    }

    template <typename T>
    double DiscreteDistribution<T>::getProbability(const T &item, const T &) const
    {
        const auto &iterator = this->probabilities_.find(item);
        return (iterator == this->probabilities_.end()) ? 0 : iterator->second;
    }

    template <typename T>
    double DiscreteDistribution<T>::setProbability(const T &item, double proba) const
    {
        assert((proba >= 0) && (proba <= 1));
        if (proba > 0)
        {
            this->probabilities[item] = proba;
        }
    }
} // namespace sdm