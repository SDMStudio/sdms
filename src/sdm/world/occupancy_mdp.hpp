#pragma once

#include <sdm/utils/config.hpp>
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

        /**
         * @brief This class provides a way to transform a Dec-POMDP into an occupancy MDP formalism.
         *
         * This problem reformulation can be used to solve the underlying Dec-POMDP with standard dynamic programming algorithms.
         *
         */
        template <class TOccupancyState = OccupancyState>
        class BaseOccupancyMDP : public BaseBeliefMDP<TOccupancyState>,
                                 public std::enable_shared_from_this<BaseOccupancyMDP<TOccupancyState>>
        {
        public:
                BaseOccupancyMDP();
                BaseOccupancyMDP(Config config);
                BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, Config config);
                BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, number memory = -1,
                                 bool store_states = true, bool store_actions = true, int batch_size = 0);

                ~BaseOccupancyMDP();

                void initialize(number memory);

                void setStateType(const StateType &state_type);

                /** @brief Get the address of the underlying MPOMDP */
                virtual std::shared_ptr<MPOMDPInterface> getUnderlyingMPOMDP() const;

                /** @brief Get the address of the underlying BeliefMDP */
                virtual std::shared_ptr<BeliefMDPInterface> getUnderlyingBeliefMDP();

                /**
                 * @brief Get the observation space of the central planner.
                 *
                 * @param t the timestep
                 * @return the space of observation of the central planner.
                 *
                 * Depending of the case, the central planner may observe or not what agents observe.
                 *
                 */
                virtual std::shared_ptr<Space> getObservationSpaceAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
                virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
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

                virtual std::shared_ptr<State> reset();
                virtual std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                virtual std::shared_ptr<Action> getRandomAction(const std::shared_ptr<State> &observation, number t);
                virtual std::shared_ptr<Action> computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t);

                std::shared_ptr<BaseOccupancyMDP<TOccupancyState>> getptr();

                // *****************
                // Temporary methods
                // *****************

                // void setInitialState(const std::shared_ptr<State> &state);
                double getRewardBelief(const std::shared_ptr<BeliefInterface> &state, const std::shared_ptr<Action> &action, number t);
                virtual std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;
                virtual Pair<std::shared_ptr<State>, double> computeNextStateAndProbability(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);

        protected:
                /** @brief The underlying well defined MPOMDP */
                std::shared_ptr<MPOMDPInterface> underlying_mpomdp;

                /** @brief Keep a pointer on the associated belief mdp that is used to compute next beliefs. */
                std::shared_ptr<BeliefMDP> belief_mdp_;

                /** @brief Initial history. */
                std::shared_ptr<HistoryInterface> initial_history_;

                /** @brief Apply compression. */
                bool compression_ = true;

                /** @brief The type of states used in the transitions. */
                StateType state_type = COMPRESSED;

                /** @brief Length of the memory */
                number memory = 0;

                /**
                 * @brief Compute the state transition in order to return next state and associated probability.
                 *
                 * This function can be modified in an inherited class to define a belief MDP with a different representation of the belief state.
                 * (i.e. BaseOccupancyMDP inherits from BaseBeliefMDP with TBelief = OccupancyState)
                 *
                 * @param occupancy_state the occupancy state
                 * @param action the action
                 * @param observation the observation
                 * @param t the timestep
                 * @return the couple (next state, transition probability in the next state)
                 */
                virtual Pair<std::shared_ptr<State>, double> computeExactNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                virtual Pair<std::shared_ptr<State>, double> computeExactNextCompressedState(const std::shared_ptr<OccupancyStateInterface> &compressed_occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, const std::shared_ptr<Observation> &observation, number t);
                virtual Pair<std::shared_ptr<State>, double> computeExactNextUncompressedState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);

                virtual Pair<std::shared_ptr<State>, double> computeSampledNextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
                virtual Pair<std::shared_ptr<OccupancyStateInterface>, double> finalizeNextState(const std::shared_ptr<OccupancyStateInterface> &next_one_step_left_compressed_occupancy_state, const std::shared_ptr<OccupancyStateInterface> &next_fully_uncompressed_occupancy_state, number t);

                virtual std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);

                virtual void updateOccupancyStateProba(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<BeliefInterface> &belief, double probability);
        };

        using OccupancyMDP = BaseOccupancyMDP<OccupancyState>;
} // namespace sdm

#include <sdm/world/occupancy_mdp.tpp>
