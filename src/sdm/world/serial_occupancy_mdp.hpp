#pragma once

#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/core/state/jhistory_tree.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class SerialOccupancyMDP : public OccupancyMDP
        {
        public:
                SerialOccupancyMDP();
                SerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp,
                                                number memory = -1,
                                                bool compression = true,
                                                bool store_states = true,
                                                bool store_action_spaces = true,
                                                int batch_size = 0);
                std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
        };
} // namespace sdm
