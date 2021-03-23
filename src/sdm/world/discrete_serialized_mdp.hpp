#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>

namespace sdm
{

    class DiscreteMMDP;

    template <typename oState = SerializedState<number>,
              typename oAction = DeterministicDecisionRule<HistoryTree_p<number>, number>> // Probablement que les actions ne seront pas bonnes
    class DiscreteSerializedMDP : public SolvableByHSVI<oState, oAction>
    {
    protected:
        std::shared_ptr<DiscreteMMDP> mmdp_;
        oState istate_;
        oState cstate_;

    public:
        using state_type = oState;
        using action_type = oAction;
        // using observation_type = oObservation;

        DiscreteSerializedMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp);
        //SerializedOccupancyMDP(std::shared_ptr<DiscreteMDP> underlying_mdp, number hist_length);
        DiscreteSerializedMDP(std::string underlying_mmdp);
        //SerializedOccupancyMDP(std::string underlying_mdp, number hist_length);

        oState &getState();

        std::shared_ptr<Reward> getReward() const;
        
        double getDiscount() { return this->mmdp_->getDiscount(); }
        double getDiscount(int t) const;

        void setDiscount(double discount) { return this->mmdp_->setDiscount(discount); }

        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const oState &);
        double getReward(const oState &ostate, const oAction &oaction) const;
        oState getInitialState();
        double getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t = 0) const;
        oState nextState(const oState &ostate, const oAction &oaction, int t = 0, HSVI<oState, oAction> *hsvi = nullptr) const;

        int getNumberAgent() const;
    };
} // namespace sdm
#include <sdm/world/discrete_serialized_mdp.tpp>