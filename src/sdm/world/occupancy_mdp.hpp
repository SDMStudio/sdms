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
#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/belief_mdp.hpp>

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
     * @tparam oState the occupancy state type 
     * @tparam oAction the occupancy action type 
     */
    template <typename oState = OccupancyState<number, JointHistoryTree_p<number>>, typename oAction = JointDeterministicDecisionRule<HistoryTree_p<number>, number>>
    class OccupancyMDP : public SolvableByHSVI<oState, oAction>,
                         public GymInterface<oState, oAction>
    {
    protected:
        oState istate_, cstate_;
        std::shared_ptr<DiscreteDecPOMDP> dpomdp_;
        typename oState::jhistory_type ihistory_ = nullptr, chistory_ = nullptr;

    public:
        using state_type = oState;
        using action_type = oAction;
        // using observation_type = oObservation;

        OccupancyMDP();

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP 
         * @param hist_length the maximum length of the history
         */
        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> , number  = -1);

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP (as a filename)
         * @param hist_length the maximum length of the history
         */
        OccupancyMDP(std::string , number  = -1);

        // ---------- RL GymInterface -------------
        oState reset();
        oState &getState();
        std::tuple<oState, std::vector<double>, bool> step(oAction );

        // ---------- HSVI exact interface -------------
        bool isSerialized() const;
        DiscreteDecPOMDP *getUnderlyingProblem();

        oState getInitialState();
        oState nextState(const oState &, const oAction &, number, std::shared_ptr<HSVI<oState, oAction>>, bool) const;
        oState nextState(const oState &, const oAction &, number = 0, std::shared_ptr<HSVI<oState, oAction>>  = nullptr) const;

        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const oState &);

        double getReward(const oState &, const oAction &) const;
        double getExpectedNextValue(ValueFunction<oState, oAction> *, const oState &, const oAction &, number = 0) const;

        // ---------- Other -------------
        
        /**
         * @brief Get the corresponding Markov Decision Process. 
         * 
         * @return std::shared_ptr<DiscreteMDP> 
         */
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * @warning The above comment is wrong!!!
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();
    };
} // namespace sdm
#include <sdm/world/occupancy_mdp.tpp>