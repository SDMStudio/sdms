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

#include "core/Coordinate.hpp"

#include <sdm/types.hpp>
#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/world/gym_interface.hpp>

namespace sdm
{
    /**
     * @brief Namespace grouping all gym environments.
     */
    namespace gym
    {

        /**
         * @brief This problem 
         * 
         */
        class RobotBin : public GymInterface
        {
        public:
            using CoordState = BaseState<Coordinate>;
            using JointCoordState = BaseState<std::vector<CoordState>>;

            RobotBin(int size_x, int size_y);

            /**
             * @brief Get the action space.
             * @param observation the observation in consideration
             * @param t time step
             * @return the action space. 
             */
            std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &observation, number t);

            /**
             * @brief Get random action.
             * @param observation the observation in consideration.
             * @param t time step.
             * @return the random action. 
             */
            std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t);

            /**
             * @brief Reset the environment and return initial observation.
             * @return the initial observation
             */
            std::shared_ptr<Observation> reset();

            /**
             * @brief Do a step on the environment.
             * @param action the action to execute
             * @return the information produced. Include : next observation, rewards, episode done  
             */
            std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

        protected:
            /**
             * @brief Dim of the grid  
             */
            int size_x_, size_y_;

            /**
             * @brief Space of possible coordinates for the robot and the waste.
             */
            std::shared_ptr<DiscreteSpace> coord_space_;

            /**
             * @brief The space of all possible actions 
             */
            std::shared_ptr<DiscreteSpace> action_space_;

            /**
             * @brief  Coordinates of the robot and the waste.
             */
            std::shared_ptr<CoordState> coord_robot_, coord_waste_;

            /**
             * @brief Space of possible coordinates for the robot and the waste.
             */
            std::shared_ptr<MultiDiscreteSpace> state_space_;

            int getSizeX() const;
            int getSizeY() const;
            std::shared_ptr<CoordState> getCoordinate(int x, int y, int add_to_x, int add_to_y);
            std::shared_ptr<State> getJointCoordinateState(const std::shared_ptr<CoordState> &coord_robot, const std::shared_ptr<CoordState> &coord_waste);
        };
    }
} // namespace sdm
