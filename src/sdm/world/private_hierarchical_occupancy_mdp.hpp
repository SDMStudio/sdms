#pragma once

#include <sdm/world/occupancy_mdp.hpp>

#include <sdm/core/state/jhistory_tree.hpp>

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
                PrivateHierarchicalOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp,
                                                number memory = -1,
                                                bool compression = true,
                                                bool store_states = true,
                                                bool store_actions = true,
                                                int batch_size = 0);
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;

        protected:
                Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeSampledNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                std::shared_ptr<State> getJointHierarchicalLabels(const std::shared_ptr<State> &joint_labels, const std::shared_ptr<State> &ostate) const;
                // std::shared_ptr<std::unordered_map<JointHistoryTree, std::shared_ptr<JointHistoryInterface>>> individual_hierarchical_history_map;
                // std::vector<std::vector<std::shared_ptr<JointHistoryInterface>>> individual_hierarchical_history_vector_vector;
        };
} // namespace sdm
