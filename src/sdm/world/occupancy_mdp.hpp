/**
 * @file occupancy_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/beliefs.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/base/base_occupancy_mdp.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{

    /**
     * @brief An occupancy MDP is a subclass of continuous state MDP where states are occupancy states. 
     * In the general case, an occupancy state refers to the whole knowledge that a central planner can have access to take decisions. But in this implementation we call occupancy state are distribution over state and joint histories .
     * 
     * @tparam TState the occupancy state type 
     * @tparam TAction the occupancy action type 
     */
    template <typename TState = OccupancyState<number, JointHistoryTree_p<number>>,
              typename TAction = JointDeterministicDecisionRule<HistoryTree_p<number>, number>>
    class OccupancyMDP : public BaseOccupancyMDP<TState, TAction>
    {
    public:
        using state_type = TState;
        using action_type = TAction;
        // using observation_type = TObservation;

        OccupancyMDP();
        OccupancyMDP(std::string dpomdp_name, number max_history_length = -1);
        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> dpomdp, number max_history_length = -1);

        void initialize(number history_length);
        std::tuple<TState, std::vector<double>, bool> step(TAction decision_rule);
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &occupancy_state);
        TState nextState(const TState &, const TAction &, number, std::shared_ptr<HSVI<TState, TAction>>, bool) const;
        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;
        double getReward(const TState &occupancy_state, const TAction &decision_rule) const;
    };

    // ##############################################################################################################################
    // ############################ SPECIALISATION FOR BeliefOccupancyState Structure ###############################################
    // ##############################################################################################################################

    /**
     * @brief Specialisation of occupancy mdp in the case of occupancy states. 
     * 
     * @tparam TActionDescriptor the action type
     * @tparam TObservation the observation type
     * @tparam TActionPrescriptor the action type (controller's one)
     */
    template <typename TActionDescriptor, typename TObservation, typename TActionPrescriptor>
    class OccupancyMDP<OccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>, TActionPrescriptor>
        : public BaseOccupancyMDP<OccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>, TActionPrescriptor>
    {
    public:
        using state_type = OccupancyState<BeliefStateGraph_p<TActionDescriptor, TObservation>, JointHistoryTree_p<TObservation>>;
        using action_type = TActionPrescriptor;
        using observation_type = TObservation;

        OccupancyMDP();
        OccupancyMDP(std::string dpomdp_name, number max_history_length = -1);
        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> dpomdp, number max_history_length = -1);

        void initialize(number history_length);
        std::tuple<state_type, std::vector<double>, bool> step(action_type decision_rule);

        std::shared_ptr<DiscreteSpace<action_type>> getActionSpaceAt(const state_type &occupancy_state);
        state_type nextState(const state_type &, const action_type &, number, std::shared_ptr<HSVI<state_type, action_type>>, bool) const;
        state_type nextState(const state_type &, const action_type &, number = 0, std::shared_ptr<HSVI<state_type, action_type>> = nullptr) const;
        double getReward(const state_type &occupancy_state, const action_type &decision_rule) const;
    };
} // namespace sdm
#include <sdm/world/occupancy_mdp.tpp>