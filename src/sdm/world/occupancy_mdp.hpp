#pragma once

#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/jhistory_tree.hpp>
#include <sdm/core/space/function_space.hpp>
#include <sdm/core/space/multi_discrete_space.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/state/occupancy_state.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
        class OccupancyMDP : public BaseBeliefMDP<OccupancyState>
        {
        public:
                OccupancyMDP();
                OccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, int batch_size = 0, bool store_action_spaces = true);

                void initialize(number memory);
                virtual std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &occupancy_state, number t = 0);
                virtual std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                virtual double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);
                double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t);
                virtual std::shared_ptr<Observation> reset();

                std::shared_ptr<State> nextOccupancyState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, const std::shared_ptr<Observation> &observation, number t = 0);
                std::shared_ptr<State> nextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr);

                double getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t);
                virtual std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;

                std::shared_ptr<MPOMDPInterface> getUnderlyingMPOMDP() const;
                std::shared_ptr<BeliefMDP> getUnderlyingBeliefMDP() const;

                // void setInitialState(const std::shared_ptr<State> &state);

        protected:
                bool compression_ = true, store_action_spaces_ = true;

                std::shared_ptr<BeliefMDP> belief_mdp_;

                std::shared_ptr<HistoryInterface> initial_history_, current_history_;

                Pair<std::shared_ptr<State>, double> computeNextStateAndProbability(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                std::shared_ptr<State> computeNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeSampledNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);

                std::shared_ptr<HistoryInterface> getNextHistory(const std::shared_ptr<Observation> &observation);
        };
} // namespace sdm
