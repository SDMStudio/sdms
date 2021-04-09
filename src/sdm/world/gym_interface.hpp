/**
 * @file gym_interface.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 04/03/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <vector>
#include <sdm/types.hpp>
#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/space/discrete_space.hpp>

namespace sdm
{
    template <typename TObservation, typename TAction, bool is_multi_agent = false>
    class GymInterface
    {
    public:
        using observation_type = TObservation;
        using action_type = TAction;

        GymInterface();

        virtual std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TObservation &) = 0;

        virtual TObservation reset() = 0;
        virtual std::tuple<TObservation, std::vector<double>, bool> step(TAction a) = 0;
    };
} // namespace sdm

#include <sdm/world/gym_interface.tpp>
