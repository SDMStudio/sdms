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
    class GymInterface
    {
    public:
        GymInterface(const std::shared_ptr<Space<std::shared_ptr<Observation>>>& observation_space, const std::shared_ptr<Space>& action_space);

        /**
         * @brief Get the action space.
         * 
         * @return the action space. 
         */
        virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &observation, number t) = 0;
        virtual std::shared_ptr<Space> getActionSpaceAt(number t) = 0;
        virtual std::shared_ptr<Space<std::shared_ptr<Observation>>> getObservationSpaceAt(number t) = 0;

        /**
         * @brief Reset the environment and return initial observation.
         * 
         * @return the initial observation
         */
        virtual std::shared_ptr<Observation> reset() = 0;

        /**
         * @brief Do a step on the environment.
         * 
         * @param action the action to execute
         * @return the information produced. Include : next observation, rewards, episode done  
         */
        virtual std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action) = 0;

    protected:

    };
} // namespace sdm

#include <sdm/world/gym_interface.tpp>
