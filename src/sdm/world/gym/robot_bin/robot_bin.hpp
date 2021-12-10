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
#include <sdm/utils/config.hpp>
#include <sdm/utils/struct/tuple.hpp>
#include <sdm/core/state/base_state.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/world/gym_interface.hpp>

namespace sdm
{
    namespace world
    {
        /**
         * @brief Namespace grouping all gym environments.
         */
        namespace gym
        {

            /**
             * @brief This environment simulates a robot moving in a grid area to collect garbages.
             *
             * Assume we instanciate a 3x3 grid environment.
             * The initial configuration is:
             *
             * |       | **0** | **1** | **2** |
             * | ----- | ----- | ----- | ----- |
             * | **0** | R     | X     | X     |
             * | **1** | X     | X     | X     |
             * | **2** | X     | X     | G     |
             *
             * where R is the location of the robot and G the location of the garbage.
             * The goal for the robot is to reach the G point.
             *
             */
            class RobotBin : public GymInterface
            {
            public:
                using CoordState = BaseState<Coordinate>;
                using JointCoordState = BaseState<std::vector<CoordState>>;

                /**
                 * @brief Construct a RobotBin environment.
                 *
                 * This environment is conform to Gym interface and thus can be used by learning algorithms.
                 *
                 * @param size_x the number of lines that contains the grid
                 * @param size_y the number of columns that contains the grid
                 */
                RobotBin(int size_x = 5, int size_y = 5);
                RobotBin(Config config = Config({{"size_x", 3}, {"size_y", 3}}));

                /**
                 * @brief Get the action space.
                 * @param observation the observation in consideration
                 * @param t the time step
                 * @return the action space.
                 */
                std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &observation, number t);

                /**
                 * @brief Get random action.
                 * @param observation the observation in consideration.
                 * @param t time step.
                 * @return the random action.
                 */
                std::shared_ptr<Action> getRandomAction(const std::shared_ptr<State> &observation, number t);

                /**
                 * @brief Reset the environment and return initial observation.
                 * @return the initial observation
                 */
                std::shared_ptr<State> reset();

                /**
                 * @brief Do a step on the environment.
                 * @param action the action to execute
                 * @return the information produced. Include : next observation, rewards, episode done
                 */
                std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

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
                std::shared_ptr<CoordState> coord_robot_, coord_garbage_;

                /**
                 * @brief Space of possible coordinates for the robot and the waste.
                 */
                std::shared_ptr<MultiDiscreteSpace> state_space_;

                /** @brief Get the number of lines */
                int getSizeX() const;
                /** @brief Get the number of columns */
                int getSizeY() const;
                /** @brief Get coordinate state relative to a point */
                std::shared_ptr<CoordState> getCoordinate(int x, int y, int add_to_x, int add_to_y);
                /** @brief Get the joint coordinate state relative robot coordinates and garbage coordinates */
                std::shared_ptr<State> getJointCoordinateState(const std::shared_ptr<CoordState> &coord_robot, const std::shared_ptr<CoordState> &coord_garbage);
            };
        } // namespace gym
    } // namespace world
} // namespace sdm
