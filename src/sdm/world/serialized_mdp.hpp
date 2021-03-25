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

    template <typename oState = SerializedState<number, number>,
              typename oAction = number>
    class SerializedMDP : public SolvableByHSVI<oState, oAction>
    {
    protected:
        std::shared_ptr<DiscreteMMDP> mmdp_;
        oState istate_;

    public:
        using state_type = oState;
        using action_type = oAction;
        // using observation_type = oObservation;

        SerializedMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp);
        //SerializedOccupancyMDP(std::shared_ptr<DiscreteMDP> underlying_mdp, number hist_length);
        SerializedMDP(std::string underlying_mmdp);
        //SerializedOccupancyMDP(std::string underlying_mdp, number hist_length);

        oState &getState();
        double getDiscount(int t) const;
        
        bool isSerialized() const;
        DiscreteMMDP *getUnderlyingProblem();

        oState getInitialState();
        oState nextState(const oState &ostate, const oAction &oaction, int t = 0, HSVI<oState, oAction> *hsvi = nullptr) const;
        
        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const oState &);
        
        double getReward(const oState &ostate, const oAction &oaction) const;
        double getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t = 0) const;

    };
} // namespace sdm
#include <sdm/world/serialized_mdp.tpp>