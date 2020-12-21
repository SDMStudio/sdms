/**
 * @file abstract_bound.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 0.1
 * @date 18/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/world/posg.hpp>
#include <sdm/utils/struct/recursive_map.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

    struct AbstractBound
    {
    protected:
        std::shared_ptr<POSG> problem_;

    public:
        virtual void prune() = 0;
        virtual void initialize() = 0;
    };
} // namespace sdm