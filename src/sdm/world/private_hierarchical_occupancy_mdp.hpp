#pragma once

#include <sdm/world/occupancy_mdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class PrivateHierarchicalOccupancyMDP : public OccupancyMDP
        {
        public:
                PrivateHierarchicalOccupancyMDP();
                PrivateHierarchicalOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, int batch_size = 0);
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
        protected:
        };
} // namespace sdm
