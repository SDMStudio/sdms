#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>

namespace sdm
{

    class DiscreteDecPOMDP;

    template <typename oState = SerializedOccupancyState<number, JointHistoryTree_p<number>>,
              typename oAction = DeterministicDecisionRule<HistoryTree_p<number>, number>>
    class SerializedOccupancyMDP : public SolvableByHSVI<oState, oAction>
    {
    protected:
        std::shared_ptr<DiscreteDecPOMDP> dpomdp_;
        oState istate_;
        oState cstate_;

    public:
        using state_type = oState;
        using action_type = oAction;
        // using observation_type = oObservation;

        SerializedOccupancyMDP();
        SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp);
        SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp, number hist_length);
        SerializedOccupancyMDP(std::string underlying_dpomdp);
        SerializedOccupancyMDP(std::string underlying_dpomdp, number hist_length);

        oState &getState();
        double getDiscount(int t) const;

        bool isSerialized() const;
        DiscreteDecPOMDP *getUnderlyingProblem();

        oState getInitialState();
        oState nextState(const oState &ostate, const oAction &oaction, int t = 0, HSVI<oState, oAction> *hsvi = nullptr) const;

        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const oState &);

        double getReward(const oState &ostate, const oAction &oaction) const;
        double getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t = 0) const;

        std::shared_ptr<SerializedMDP<>> toMDP();
        // std::shared_ptr<DiscretePOMDP> toPOMDP();
    };
} // namespace sdm
#include <sdm/world/serialized_occupancy_mdp.tpp>