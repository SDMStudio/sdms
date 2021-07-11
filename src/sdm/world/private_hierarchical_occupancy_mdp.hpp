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
                PrivateHierarchicalOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, int batch_size = 0, bool store_action_spaces = true);
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;
        protected:
                Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeSampledNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                std::shared_ptr<State> getJointHierarchicalLabels(const std::shared_ptr<State> &joint_labels) const;
        };
} // namespace sdm
