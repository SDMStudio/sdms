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
        void logo();
        std::default_random_engine &global_urng();
        std::string getState(state);
        std::string getAgentActionState(agent, action, state);

    } // namespace common
} // namespace sdm
