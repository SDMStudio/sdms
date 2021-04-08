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
     * @tparam TBelief refer to the serialized state type
     * @tparam TAction refer to the number type
     */
    template <typename TBelief = SerializedBeliefState,
              typename TAction = number,
              typename TObservation = number>
    class SerializedBeliefMDP : public BeliefMDP<TBelief, TAction, TObservation>,
                                public std::enable_shared_from_this<SerializedBeliefMDP<TBelief, TAction,TObservation>>
    {
    protected:
        std::shared_ptr<DiscretePOMDP> mpomdp_;

    public:
        using state_type = TBelief;
        using action_type = TAction;

        SerializedBeliefMDP();
        SerializedBeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_mmdp);
        SerializedBeliefMDP(std::string underlying_mmdp);

        std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> getptr();

        TBelief &getState();
        double getDiscount(int t) const;

        bool isSerialized() const;
        DiscretePOMDP *getUnderlyingProblem();

        TBelief getInitialState();
        TBelief nextState(const TBelief &ostate, const TAction &oaction, int t = 0, HSVI<TBelief, TAction> *hsvi = nullptr) const;

        //Tempo à vérifier leur utilité
        TBelief nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const;
        double getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const;



        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TBelief &);

        double getReward(const TBelief &ostate, const TAction &oaction) const;
        double getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, const TBelief &ostate, const TAction &oaction, int t = 0) const;

        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> toBeliefMDP();
    };
} // namespace sdm
#include <sdm/world/serialized_belief_mdp.tpp>