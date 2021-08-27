#pragma once

#include <sdm/world/mdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>


// #include <sdm/world/discrete_mmdp.hpp>
// #include <sdm/world/discrete_pomdp.hpp>
// #include <sdm/world/discrete_decpomdp.hpp>
// #include <sdm/world/networked_distributed_pomdp.hpp>

// #include <sdm/world/solvable_by_hsvi.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/world/serialized_mmdp.hpp>
// #include <sdm/world/serialized_occupancy_mdp.hpp>
// #include <sdm/world/serialized_belief_mdp.hpp>

namespace sdm
{
    
    /**
     * @brief Namespace grouping functions to manipulate problems.
     */
    namespace world
    {
        /**
         * @brief Get the list of available worlds. 
         * Usage:
         * 
         *          std::cout << sdm::world::available() << std::endl;
         * 
         * @return the list of available worlds.
         */
        std::vector<std::string> available()
        {
            return {"MDP", "POMDP", "MMDP", "DecPOMDP", "Extensive-MDP", "Extensive-POMDP", "Extensive-DecPOMDP"};
        }
    } // namespace world
    
} // namespace sdm
