#pragma once

#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_mmdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/world/ndpomdp.hpp>

#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/serialized_occupancy_mdp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

namespace sdm
{
    namespace world
    {
        
        /**
         * @brief Get all available algorithms.
         * 
         * @return the list of available algorithms.
         */
        std::vector<std::string> available()
        {
            return {"MDP", "POMDP", "MMDP", "DecPOMDP", "Extensive-DecPOMDP"};
        }
    } // namespace world
    
} // namespace sdm
