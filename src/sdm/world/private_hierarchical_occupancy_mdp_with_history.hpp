#pragma once

#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class PrivateHierarchicalOccupancyMDPWithHistory : public PrivateHierarchicalOccupancyMDP
        {
        public:
                PrivateHierarchicalOccupancyMDPWithHistory();
                PrivateHierarchicalOccupancyMDPWithHistory(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, bool compression = true, bool store_states = true, bool store_actions = true, int batch_size = 0);
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                std::shared_ptr<Observation> reset();
                double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);
                std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &occupancy_state, number t = 0);
                // std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;
        protected:
                std::shared_ptr<HistoryInterface> getJointLabel(const std::shared_ptr<HistoryInterface> &joint_history, const std::shared_ptr<State> &occupancy_state);
        };
} // namespace sdm
