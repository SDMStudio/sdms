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
        template <class TOccupancyState = OccupancyState>
        class BaseOccupancyMDP : public BaseBeliefMDP<TOccupancyState>
        {
        public:
                BaseOccupancyMDP();
                BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1, bool compression = true, bool store_states = true, bool store_actions = true, bool generate_action_spaces = false, int batch_size = 0);

                void initialize(number memory);

                /**
                 * @brief Get the next occupancy state.
                 * This function returns the next occupancy state. To do so, we check in the MDP graph the existance of an edge (action / observation) starting from the current occupancy state. 
                 * If it exists, we return the associated next occupancy state. Otherwise, we compute the next occupancy state using  "computeNextStateAndProbability" function and add the edge from the current occupancy state to the next occupancy state in the graph.
                 * 
                 * @param occupancy state the occupancy state
                 * @param action the action
                 * @param observation the observation
                 * @param t the timestep
                 * @return the next occupancy state
                 */
                virtual std::shared_ptr<State> nextOccupancyState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, const std::shared_ptr<Observation> &observation, number t = 0);

                /** @brief Get the address of the underlying MPOMDP */
                virtual std::shared_ptr<MPOMDPInterface> getUnderlyingMPOMDP() const;

                /** @brief Get the address of the underlying BeliefMDP */
                virtual std::shared_ptr<BeliefMDP> getUnderlyingBeliefMDP() const;

                /**
                 * @brief Get the observation space of the central planner. 
                 * Depending of the case, the central planner may observe or not what agents observe.
                 * 
                 * @param t the timestep
                 * @return the space of observation of the central planner. 
                 */
                virtual std::shared_ptr<Space> getObservationSpace(number t);
                virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
                virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &occupancy_state, number t = 0);
                virtual double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0);
                virtual bool checkCompatibility(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Observation> &observation);

                // **********************
                // SolvableByHSVI methods
                // **********************

                // std::shared_ptr<State> nextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr);
                // double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t);
                virtual double do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon);

                // *****************
                //    RL methods
                // *****************

                virtual std::shared_ptr<Observation> reset();
                virtual std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                virtual std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t);
                virtual std::shared_ptr<Action> computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t);

                // *****************
                // Temporary methods
                // *****************

                // void setInitialState(const std::shared_ptr<State> &state);
                double getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t);
                virtual std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;

                // *****************
                //    PROFILING
                // *****************

                static double TIME_IN_NEXT_STATE, TIME_IN_COMPRESS, TIME_IN_GET_ACTION, TIME_IN_STEP, TIME_IN_GET_REWARD, TIME_IN_NEXT_OSTATE, TIME_IN_EXP_NEXT;
                static double TIME_IN_UNDER_STEP, TIME_IN_APPLY_DR;
                static number PASSAGE_IN_NEXT_STATE;
                static unsigned long MEAN_SIZE_STATE;

                /** @brief Initial and current histories. */
                std::shared_ptr<HistoryInterface> initial_history_, current_history_;

        protected:
                /** @brief Hyperparameters. */
                bool compression_ = true, generate_action_spaces_ = false;

                /** @brief Keep a pointer on the associated belief mdp that is used to compute next beliefs. */
                std::shared_ptr<BeliefMDP> belief_mdp_;

                /**
                 * @brief Compute the state transition in order to return next state and associated probability.
                 * This function can be modified in an inherited class to define a belief MDP with a different representation of the belief state. 
                 * (i.e. BaseOccupancyMDP inherits from BaseBeliefMDP with TBelief = OccupancyState)
                 * 
                 * @param belief the belief
                 * @param action the action
                 * @param observation the observation
                 * @param t the timestep
                 * @return the couple (next state, transition probability in the next state)
                 */
                virtual Pair<std::shared_ptr<State>, double> computeNextStateAndProbability(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0
                virtual std::shared_ptr<State> computeNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeSampledNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);

                std::shared_ptr<HistoryInterface> getNextHistory(const std::shared_ptr<Observation> &observation);

                virtual std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);

                /** @brief Return true if compression must be done */
                virtual bool do_compression(number t) const;

                virtual void update_occupancy_state_proba(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double probability);

                std::shared_ptr<std::unordered_map<JointDeterministicDecisionRule, std::shared_ptr<Action>>> action_map_;

                // std::shared_ptr<Action> getActionPointer(std::shared_ptr<Action> action_tmp);
        };

        using OccupancyMDP = BaseOccupancyMDP<OccupancyState>;
} // namespace sdm

#include <sdm/world/occupancy_mdp.tpp>
