#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>

namespace sdm
{

    class DiscreteDecPOMDP;

    /**
     * @brief An Serialized occupancy MDP is a subclass of continuous state MDP where states are seriliazed occupancy states and the resolution is serialized. 
     * In the general case, a Serialized occupancy state refers to the knowledge that a central planner can have access to take decisions at a precise agent. 
     * But in this implementation we call serialized occupancy state a distribution over serialized state and joint histories .
     * 
     * @tparam oState refers to an serialized occupancy state type 
     * @tparam oAction refers to a occupancy action type 
     */
    template <typename oState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>,
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

        /**
         * @brief Get the corresponding Belief Markov Decision Process. It corresponds to the reformulation of the original POMP in a MDP where the state space is the space of beliefs. 
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();   
    };
} // namespace sdm
#include <sdm/world/serialized_occupancy_mdp.tpp>