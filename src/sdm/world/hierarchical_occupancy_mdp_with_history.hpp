#pragma once

#include <sdm/world/hierarchical_occupancy_mdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class HierarchicalOccupancyMDPWithHistory : public HierarchicalOccupancyMDP
        {
        public:
                HierarchicalOccupancyMDPWithHistory();
                HierarchicalOccupancyMDPWithHistory(const std::shared_ptr<HierarchicalMPOMDP> &hierarchical_mpomdp, number memory = -1, bool compression = true, bool store_states = true, bool store_actions = true, bool generate_action_spaces = false, int batch_size = 0);
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                std::shared_ptr<Observation> reset();
                double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);
                std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t);
        protected:
                // std::shared_ptr<HistoryInterface> getJointLabel(const std::shared_ptr<HistoryInterface> &joint_history, const std::shared_ptr<State> &occupancy_state);
        };
} // namespace sdm
