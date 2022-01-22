/**
 * @file action.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for interface action class
 * @version 0.1
 * @date 11/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/item.hpp>

namespace sdm
{
    template <typename T>
    class Distribution : public std::enable_shared_from_this<Distribution<T>>
    {
    public:
        virtual T sample() const = 0;
        virtual double getProbability(const T &begin, const T &end) const = 0;
        // std::shared_ptr<DiscreteDistribution<T>> toDiscreteDistribution();
    };

    template <typename T>
    class DiscreteDistribution : public Distribution<T>
    {
    public:
        DiscreteDistribution();
        DiscreteDistribution(const DiscreteDistribution<T>& copy);
        virtual ~DiscreteDistribution() {}
        virtual T sample() const;

        virtual double getProbability(const T &begin, const T & = 0) const;

        virtual void setProbability(const T &item, double proba);

    protected:
        // typedef typename bimap_item_index<T>::value_type bimap_pair;
        // bimap_item_index<T> bimap_item_to_index_;
        // std::vector<double> probabilities_;
        std::unordered_map<T, double> probabilities;
    };
} // namespace sdm
#include <sdm/core/distribution.tpp>