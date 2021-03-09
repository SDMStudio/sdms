/**
 * @file belief_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that contains the implementation of the belief mdp process class.
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

namespace sdm
{
    class DiscretePOMDP;

    /**
     * @brief The BeliefMDP class is the interface that enables solving Discret POMDP using HSVI algorithm.
     * 
     * @tparam TBelief the belief type
     * @tparam TAction the action type
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

        double getReward(const TBelief &belief, const TAction &action) const;

        double getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, const TBelief &belief, const TAction &action, int t) const;

        /**
         * @brief Get the Observation Probability p(o | b, a)
         */
        double getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const;

        TBelief reset();
    };
} // namespace sdm
#include <sdm/world/belief_mdp.tpp>