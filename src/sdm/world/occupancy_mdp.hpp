#pragma once

#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/occupancy_state.hpp>

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
                OccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, bool store_action_spaces = true);

                void initialize(number history_length);

                /**
                 * @brief Get the next occupancy state.
                 * This function returns the next occupancy state. To do so, we check in the MDP graph the existance of an edge (action / observation) starting from the current occupancy state. 
                 * If exists, we return the associated next occupancy state. Otherwise, we compute the next occupancy state using  "computeNextStateAndProba" function and add the edge from the current occupancy state to the next occupancy state in the graph.
                 * 
                 * @param occupancy state the occupancy state
                 * @param action the action
                 * @param observation the observation
                 * @param t the timestep
                 * @return the next occupancy state
                 */
                std::shared_ptr<State> nextOccupancyState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, const std::shared_ptr<Observation> &observation, number t = 0);

                /** @brief Get the address of the underlying MPOMDP */
                std::shared_ptr<MPOMDPInterface> getUnderlyingMPOMDP() const;

                /** @brief Get the address of the underlying BeliefMDP */
                std::shared_ptr<BeliefMDP> getUnderlyingBeliefMDP() const;

                // **********************
                // SolvableByHSVI methods
                // **********************

                std::shared_ptr<State> nextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr);
                std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);
                double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t);

                // *****************
                //    RL methods
                // *****************

                std::shared_ptr<Observation> reset();
                std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &occupancy_state, number t = 0);
                std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t);

                // *****************
                // Temporary methods
                // *****************

                // void setInitialState(const std::shared_ptr<State> &state);
                double getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t);
                std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;

        protected:
                /** @brief Hyperparameters. */
                bool compression_ = true, store_action_spaces_ = true;

                /** @brief Keep a pointer on the associated belief mdp that is used to compute next beliefs. */
                std::shared_ptr<BeliefMDP> belief_mdp_;

                /** @brief Initial and current histories. */
                std::shared_ptr<HistoryInterface> initial_history_, current_history_;

                /**
                 * @brief Compute the state transition in order to return next state and associated probability.
                 * This function can be modify in an inherited class to define a belief MDP with a different representation of the belief state. 
                 * (i.e. OccupancyMDP inherit from BaseBeliefMDP with TBelief = OccupancyState)
                 * 
                 * @param belief the belief
                 * @param action the action
                 * @param observation the observation
                 * @param t the timestep
                 * @return the couple (next state, transition probability in the next state)
                 */
                virtual Pair<std::shared_ptr<State>, double> computeNextStateAndProba(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
        };
} // namespace sdm
