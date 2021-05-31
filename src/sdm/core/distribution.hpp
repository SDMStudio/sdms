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
    class Distribution
    {
        virtual T sample() const = 0;
        virtual double getProbability(const T &begin, const T &end = 0) const = 0;
    };
} // namespace sdm