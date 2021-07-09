#pragma once

#include <sdm/world/occupancy_mdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class SampledOccupancyMDP : public OccupancyMDP
        {
        public:
                SampledOccupancyMDP();
                SampledOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, int batch_size = 16, bool store_action_spaces = true);
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);
        protected:
                int batch_size_;
                double sampled_reward_;
                bool is_done_;
                std::shared_ptr<State> next_observation_;
                Pair<std::shared_ptr<State>, double> computeNextStateAndProba(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
        };
} // namespace sdm
