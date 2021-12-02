#pragma once

//  ------------------------------------------------------------------------
// |                 INCLUDE WORLDS INTERFACES                              |
//  ------------------------------------------------------------------------

#include <sdm/world/base/mdp_interface.hpp>
#include <sdm/world/base/mmdp_interface.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/world/base/belief_mdp_interface.hpp>
#include <sdm/world/base/ndpomdp_interface.hpp>

#include <sdm/world/gym_interface.hpp>
#include <sdm/world/bayesian_game_interface.hpp>
#include <sdm/world/solvable_by_dp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/solvable_by_mdp.hpp>

//  ------------------------------------------------------------------------
// |               INCLUDE WORLDS IMPLEMENTATIONS                           |
//  ------------------------------------------------------------------------

#include <sdm/world/mdp.hpp>
#include <sdm/world/pomdp.hpp>
#include <sdm/world/mmdp.hpp>
#include <sdm/world/mpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>

//  ------------------------------------------------------------------------
// |               INCLUDE SERIAL WORLDS                                    |
//  ------------------------------------------------------------------------

#include <sdm/world/serial_mmdp.hpp>
#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/world/serial_occupancy_mdp.hpp>


//  ------------------------------------------------------------------------
// |               INCLUDE HIERARCHICAL WORLDS                              |
//  ------------------------------------------------------------------------

#include <sdm/world/hierarchical_mpomdp.hpp>
#include <sdm/world/hierarchical_occupancy_mdp.hpp>


//  ------------------------------------------------------------------------
// |               INCLUDE BAYESIAN GAMES                                   |
//  ------------------------------------------------------------------------

#include <sdm/world/two_players_bayesian_game.hpp>
#include <sdm/world/two_players_normal_form_game.hpp>


//  ------------------------------------------------------------------------
// |                  INCLUDE WORLD REGISTRY                                |
//  ------------------------------------------------------------------------

#include <sdm/world/registry.hpp>


// namespace sdm
// {

//     /**
//      * @brief Namespace grouping functions to manipulate problems.
//      */
//     namespace world
//     {
//         /**
//          * @brief Get the list of available worlds.
//          *
//          * Usage :
//          *
//          * ```cpp
//          * std::cout << sdm::world::available() << std::endl;
//          * ```
//          *
//          * @return the list of available worlds.
//          */
//         std::vector<std::string> available()
//         {
//             return {"MDP", "BeliefMDP", "OccupancyMDP", "Extensive-MDP", "Extensive-BeliefMDP", "Extensive-OccupancyMDP", "Hierarchical-MDP", "Hierarchical-BeliefMDP", "Hierarchical-OccupancyMDP"};
//         }

//     } // namespace world

// } // namespace sdm
