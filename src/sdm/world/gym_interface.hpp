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
    /**
     * @brief The GymInterface is a formalism used by environments simulation.
     * 
     * @tparam TObservation observation type
     * @tparam TAction action type
     * @tparam is_multi_agent set to true if the environment is multi agent.
     */
    template <typename TObservation, typename TAction, bool is_multi_agent = false>
    class GymInterface
    {
    public:
        using observation_type = TObservation;
        using action_type = TAction;

        GymInterface();

        /**
         * @brief Get the action space.
         * 
         * @return the action space. 
         */
        virtual std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TObservation &) = 0;

        /**
         * @brief Reset the environment and return initial observation.
         * 
         * @return the initial observation
         */
        virtual TObservation reset() = 0;

        /**
         * @brief Do a step on the environment.
         * 
         * @param action the action to execute
         * @return the information produced. Include : next observation, rewards, episode done  
         */
        virtual std::tuple<TObservation, std::vector<double>, bool> step(TAction action) = 0;
    };
} // namespace sdm

#include <sdm/world/gym_interface.tpp>
