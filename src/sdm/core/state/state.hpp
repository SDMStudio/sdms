/**
 * @file state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for state class
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
    class Observation : public Item
    {
    public :
        virtual std::string str() const = 0;
    };

    class State : public Observation
    {
    public :
        virtual std::string str() const = 0;
    };
} // namespace sdm