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
                BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &decpomdp, Config config);
                BaseOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &decpomdp, int memory = -1,
                                 bool store_states = true, bool store_actions = true, int batch_size = 0);

                ~BaseOccupancyMDP();

                /**
                 * @brief Set the type of states.
                 * 
                 * The type of state can be one of: compressed, one step uncompressed, fully uncompressed, 
                 * compressed keep all, one step uncompressed keep all. The "keep all" versions are suited 
                 * when the value function structure requires full knowledge of the state.
                 * 
                 */
                virtual void setStateType(const StateType &state_type);

                /** @brief Get the address of the underlying MPOMDP */
                virtual std::shared_ptr<MPOMDPInterface> getUnderlyingMPOMDP() const;

                /** @brief Get the address of the underlying BeliefMDP */
                virtual std::shared_ptr<BeliefMDPInterface> getUnderlyingBeliefMDP();

                /**
                 * @brief Get the observation space of the central planner.
                 * 
                 * Depending of the case, the central planner may see what agents see.
                 *
                 * @param t the timestep
                 * @return the space of observation of the central planner.
                 *
                 */
                virtual std::shared_ptr<Space> getObservationSpaceAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

                /**
                 * @brief Get the action space of the central planner.
                 * 
                 * In this case, actions are decision rules (deterministic or stochastic).
                 *
                 * @param occupancy_state the occupancy state used to get compute the available actions
                 * @param t the timestep
                 * @return the space of actions of the central planner.
                 *
                 */
                virtual std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);

                /**
                 * @brief Check whether the central planner observation is compatible with an underlying observation.
                 */
                virtual bool checkCompatibility(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Observation> &observation);

                // **********************
                // SolvableByHSVI methods
                // **********************

                virtual double do_excess(double incumbent, double lb, double ub, double cost_so_far, double error, number horizon);

                // *****************
                //    RL methods
                // *****************

                virtual std::shared_ptr<State> reset();
                virtual std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
                virtual std::shared_ptr<Action> getRandomAction(const std::shared_ptr<State> &observation, number t);
                virtual std::shared_ptr<Action> computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t);

                std::shared_ptr<BaseOccupancyMDP<TOccupancyState>> getptr();

        protected:
                /** @brief The underlying well defined MPOMDP */
                std::shared_ptr<MPOMDPInterface> decpomdp;

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

                virtual std::shared_ptr<Space> computeActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
        };

        using OccupancyMDP = BaseOccupancyMDP<OccupancyState>;
} // namespace sdm

#include <sdm/world/occupancy_mdp.tpp>
