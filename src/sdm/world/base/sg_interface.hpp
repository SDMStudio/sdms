/**
 * @file discrete_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the MDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/base/sg_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Stochastic Game.
     * 
     */
    class SGInterface : virtual public MMDPInterface
    {
    public:
        /**
         * @brief Get the reward at timestep t when executing an action in a specific state.
         * 
         * @param state the current state
         * @param action the action
         * @param t the timestep
         * @return double the reward for each agent
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;

        /**
         * @brief Get the reward at timestep t when executing an action in a specific state.
         * 
         * @param state the current state
         * @param action the action
         * @param t the timestep
         * @return double the reward for each agent
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number agent_id, number t) const = 0;
    };

} // namespace sdm
