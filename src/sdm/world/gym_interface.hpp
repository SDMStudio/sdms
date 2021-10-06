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
    class GymInterface
    {
    public:
        virtual ~GymInterface() {}

        /**
         * @brief Get the action space.
         * @param observation the observation in consideration
         * @param t time step
         * @return the action space. 
         */
        virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &observation, number t) = 0;

        /**
         * @brief Get random action.
         * @param observation the observation in consideration.
         * @param t time step.
         * @return the random action. 
         */
        virtual std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t) = 0;

        /**
         * @brief Reset the environment and return initial observation.
         * @return the initial observation
         */
        virtual std::shared_ptr<Observation> reset() = 0;

        /**
         * @brief Do a step on the environment.
         * @param action the action to execute
         * @return the information produced. Include : next observation, rewards, episode done  
         */
        virtual std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action) = 0;
    };
} // namespace sdm
