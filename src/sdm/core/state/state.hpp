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
    public:
        virtual ~Observation() {}
        virtual std::string str() const = 0;
    };

    class BeliefInterface;

    class State : public Observation
    {
    public:
        virtual ~State() {}

        virtual std::shared_ptr<BeliefInterface> toBelief();

        virtual std::string str() const = 0;

        bool operator==(const State &sp) const
        {
            return (this->str() == sp.str());
        }
        bool operator!=(const State &sp) const 
        {
            return !(this->operator==(sp));
        }
    };
} // namespace sdm