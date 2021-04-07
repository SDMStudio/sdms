#pragma once

#include <sdm/types.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_belief_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>

namespace sdm
{

    class DiscretePOMDP;

    /**
     * @brief An Serialized MDP is a subclass of POMDP where belief are serialized beliefs. 
     * In the general case, a serialized belief refers to the whole knowledge that a central planner can have access to take decisions at the time step of a precise agent. 
     * 
     * @tparam oState refer to the serialized state type
     * @tparam oAction refer to the number type
     */
    template <typename TBelief = SerializedBeliefState,
              typename TAction = number,
              typename TObservation = number>
    class SerializedBeliefMDP : public BeliefMDP<TBelief, TAction, TObservation>
    {
    protected:
        std::shared_ptr<DiscretePOMDP> mpomdp_;

    public:
        using state_type = oState;
        using action_type = oAction;

        SerializedBeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_mmdp);
        SerializedBeliefMDP(std::string underlying_mmdp);

        std::shared_ptr<SerializedBeliefMDP> getptr();

        oState &getState();
        double getDiscount(int t) const;

        bool isSerialized() const;
        DiscreteMMDP *getUnderlyingProblem();

        oState getInitialState();
        oState nextState(const oState &ostate, const oAction &oaction, int t = 0, HSVI<oState, oAction> *hsvi = nullptr) const;

        std::shared_ptr<DiscreteSpace<oAction>> getActionSpaceAt(const oState &);

        double getReward(const oState &ostate, const oAction &oaction) const;
        double getExpectedNextValue(ValueFunction<oState, oAction> *value_function, const oState &ostate, const oAction &oaction, int t = 0) const;

        // Problem conversion
        std::shared_ptr<SerializedBeliefMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();
    };
} // namespace sdm
#include <sdm/world/serialized_mdp.tpp>