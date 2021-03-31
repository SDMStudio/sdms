#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/decision_rules/det_decision_rule.hpp>

namespace sdm
{

    class DiscreteMMDP;

    /**
     * @brief An Serialized MDP is a subclass of MDP where states are serialized states. 
     * In the general case, a serialized state refers to the whole knowledge that a central planner can have access to take decisions at the time step of an precise agent. 
     * 
     * @tparam oState refer to the serialized state type
     * @tparam oAction refer to the number type
     */
    template <typename oState = SerializedState,
              typename oAction = number>
    class SerializedMDP : public SolvableByHSVI<oState, oAction>,
                          public std::enable_shared_from_this<SerializedMDP<oState, oAction>>

    {
    protected:
        std::shared_ptr<DiscreteMMDP> mmdp_;

    public:
        using state_type = oState;
        using action_type = oAction;

        SerializedMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp);
        SerializedMDP(std::string underlying_mmdp);

        std::shared_ptr<SerializedMDP> getptr();

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
        std::shared_ptr<SerializedMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();        
    };
} // namespace sdm
#include <sdm/world/serialized_mdp.tpp>