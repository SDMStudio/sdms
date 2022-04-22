/**
 * @file observation.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 27/08/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/item.hpp>

namespace sdm
{
    /**
     * @brief A public interface for observations. 
     * 
     * Any class inheriting from this interface will be considered as generic observation for algorithms.
     * Consider sections [Theoritical Background](https://aldavid.gitlabpages.inria.fr/sdms/tutorials/theory.html) and [Algorithms](https://aldavid.gitlabpages.inria.fr/sdms/tutorials/algorithms/) for more information.   
     */
    class Observation : public Item
    {
    public:
        using base = Observation;
        
        virtual ~Observation() {}
        virtual std::string str() const = 0;
    };

} // namespace sdm