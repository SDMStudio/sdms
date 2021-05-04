/**
 * @file occupancy_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{

    /**
     * @brief An occupancy MDP is a subclass of continuous state MDP where states are occupancy states. 
     * In the general case, an occupancy state refers to the whole knowledge that a central planner can have access to take decisions. 
     * But in this implementation an occupancy state refers to a distribution over states and joint histories .
     * 
     * @tparam TState the occupancy state type 
     * @tparam TAction the occupancy action type 
     */
    template <typename TState, typename TAction>
    class PrivateOccupancyMDP : public OccupancyMDP<TState, TAction>
    {
    public:
        using state_type = TState;
        using action_type = TAction;

        PrivateOccupancyMDP();

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP 
         * @param hist_length the maximum length of the history
         */
        PrivateOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length = -1);

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP (as a filename)
         * @param hist_length the maximum length of the history
         */
        PrivateOccupancyMDP(std::string underlying_dpomdp, number hist_length = -1);

        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &);
    };
} // namespace sdm
#include <sdm/world/private_occupancy_mdp.tpp>
