#pragma once

#include <sdm/world/private_hierarchical_occupancy_mdp_with_history.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class PrivateHierarchicalOccupancyMDPWithObservation : public PrivateHierarchicalOccupancyMDPWithHistory
        {
        public:
                PrivateHierarchicalOccupancyMDPWithObservation();
                PrivateHierarchicalOccupancyMDPWithObservation(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, bool compression = true, bool store_states = true, bool store_actions = true, bool generate_action_spaces = false, int batch_size = 0);
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                std::shared_ptr<Observation> reset();
                double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);
                std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t);
                std::shared_ptr<Action> computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t);
                std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &decision_rule, number t) const;
                // std::shared_ptr<Observation> getJointHierarchicalObservations(const std::shared_ptr<Observation> &observation) const;
        protected:
                std::shared_ptr<Observation> current_observation_;
                virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeSampledNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
        };
} // namespace sdm
