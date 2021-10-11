#pragma once

#include <sdm/types.hpp>
// #include <sdm/world/mdp.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/occupancy_mdp.hpp>


// #include <sdm/world/discrete_mmdp.hpp>
// #include <sdm/world/discrete_pomdp.hpp>
// #include <sdm/world/discrete_decpomdp.hpp>
// #include <sdm/world/networked_distributed_pomdp.hpp>

// #include <sdm/world/solvable_by_hsvi.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/occupancy_mdp.hpp>
// #include <sdm/world/serial_mmdp.hpp>
// #include <sdm/world/serial_occupancy_mdp.hpp>
// #include <sdm/world/serial_belief_mdp.hpp>

namespace sdm
{
    
    /**
     * @brief Namespace grouping functions to manipulate problems.
     */
    namespace world
    {
        /**
         * @brief Get the list of available worlds. 
         * 
         * Usage :
         * 
         * ```cpp
         * std::cout << sdm::world::available() << std::endl;
         * ```
         * 
         * @return the list of available worlds.
         */
        std::vector<std::string> available();
    } // namespace world
    
} // namespace sdm
