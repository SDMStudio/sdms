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
#include <sdm/core/action/hierarchical_private_joint_det_decision_rule.hpp>


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
    template <typename TState, typename TAction>
    class HierarchicalPrivateOccupancyMDP : public GymInterface<TState, TAction>
    {
    protected:

        /**
         * @brief The problem that we want solve. 
         */
        std::shared_ptr<DiscreteDecPOMDP> dpomdp_;

        /**
         * @brief Keep initial and current states.
         */
        std::shared_ptr<TState> initial_state_, current_state_;

    public:
        /**
         * @brief Keep initial and current histories.
         */
        typename TState::jhistory_type initial_history_ = nullptr, current_history_ = nullptr, previous_history_ = nullptr;
        using state_type = TState;
        using action_type = TAction;

        HierarchicalPrivateOccupancyMDP();
        HierarchicalPrivateOccupancyMDP(std::string dpomdp_name, number max_history_length = -1);
        HierarchicalPrivateOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> dpomdp, number max_history_length = -1);

        void initialize(number history_length);
        std::tuple<TState, std::vector<double>, bool> step(TAction decision_rule);
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &occupancy_state);
        TState nextState(const TState &, const TAction &, number, std::shared_ptr<HSVI<TState, TAction>>, bool) const;
        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;
        double getReward(const TState &occupancy_state, const TAction &decision_rule) const;

        TState getInitialState();

        TState reset();

        DiscreteDecPOMDP *getUnderlyingProblem();

        Joint<Joint<typename TState::ihistory_type>> getJointHierarchicalLabels(const TState &, Joint<typename TState::ihistory_type>) const;

        Joint<number> getJaction(TAction);

        void updateHistory();

        std::vector<Joint<typename TAction::output>> get_vector_lower_ranked_agents_jactions_reversed(number agent);
    };

} // namespace sdm
#include <sdm/world/hierarchical_private_occupancy_mdp.tpp>