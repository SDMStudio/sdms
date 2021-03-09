/**
 * @file belief_mdp.hpp
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
#include <sdm/world/solvable_by_hsvi.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    class DiscretePOMDP;

    /**
     * @brief 
     * 
     * @tparam TBelief 
     * @tparam TAction 
     */
    template <typename TBelief, typename TAction, typename TObservation>
    class BeliefMDP : public SolvableByHSVI<TBelief, TAction>,
                      public GymInterface<DiscreteSpace<TBelief>, DiscreteSpace<TAction>>
    {
    protected:
        std::shared_ptr<DiscretePOMDP> pomdp_;
        TBelief istate_;
        TBelief cstate_;

    public:
        using state_type = TBelief;
        using action_type = TAction;
        using observation_type = TObservation;

        BeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp);
        BeliefMDP(std::string underlying_pomdp);

        double getDiscount() { return this->pomdp_->getDiscount(); }
        void setDiscount(double discount) { return this->pomdp_->setDiscount(discount); }

        TBelief getInitialState();
        TBelief &getState();
        std::shared_ptr<Reward> getReward() const;

        TBelief nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const;
        TBelief nextState(const TBelief &belief, const TAction &action, int t = 0, HSVI<TBelief, TAction> *hsvi = nullptr) const;
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TBelief &ostate = TBelief());

        /**
         * @fn double getReward(Vector belief, number action);
         * @brief Get transformed reward from action and belief  
         */
        double getReward(const TBelief &belief, const TAction &action) const;

        /**
         * @brief Get the Expected Next Value object
         * 
         * @param value_function 
         * @param belief 
         * @param action 
         * @param t 
         * @return double 
         */
        double getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, const TBelief &belief, const TAction &action, int t) const;

        /**
         * @brief Get the Observation Probability p(o | b, a)
         * 
         * @param action 
         * @param obs 
         * @param belief 
         * @return double 
         */
        double getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const;

        TBelief reset();
    };
} // namespace sdm
#include <sdm/world/belief_mdp.tpp>