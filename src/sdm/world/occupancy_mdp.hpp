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
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{

    class DiscreteDecPOMDP;

    /**
     * @brief 
     * 
     * @tparam oState 
     * @tparam oAction 
     */
    template <typename oState = OccupancyState<number, JointHistoryTree_p<number>>, typename oAction = Joint<DeterministicDecisionRule<HistoryTree_p<number>, number>>>
    class OccupancyMDP : public SolvableByHSVI<oState, oAction>
    {
    protected:
        std::shared_ptr<DiscreteDecPOMDP> dpomdp_;
        oState istate_;
        oState cstate_;

    public:
        using state_type = oState;
        using action_type = oAction;
        // using observation_type = oObservation;

        OccupancyMDP();
        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp);
        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length);
        OccupancyMDP(std::string underlying_dpomdp);
        OccupancyMDP(std::string underlying_dpomdp, number hist_length);

        oState &getState();

        bool isSerialized() const;
        DiscreteDecPOMDP *getUnderlyingProblem();

        oState getInitialState();
        oState nextState(const oState &ostate, const oAction &oaction, int t = 0, HSVI<oState, oAction> *hsvi = nullptr) const;

        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const oState &);
        
        double getReward(const oState &ostate, const oAction &oaction) const;
        double getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t = 0) const;
    };
} // namespace sdm
#include <sdm/world/occupancy_mdp.tpp>