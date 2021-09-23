#pragma once

#include <random>
#include <sdm/types.hpp>

namespace sdm
{
    /**
     * @brief Namespace grouping all common functions to the whole project.
     */
    namespace common
    {
        /**
         * @brief Display the logo of SDM'Studio. 
         */
        void logo();

        /**
         * @brief Get the random engine. 
         */
        std::default_random_engine &global_urng();
        
        std::string getState(number state);
        std::string getAgentActionState(number agent_id, number action, number state);

    } // namespace common
} // namespace sdm
