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
#include <sdm/world/decpomdp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TBelief 
     * @tparam TAction 
     */
    template <typename TBelief, typename TAction, typename TObservation>
    class BeliefMDP : public SolvableByHSVI<TBelief, TAction, TObservation>
    {
    protected:
        std::shared_ptr<DecPOMDP> pomdp_;
        TBelief istate_;
        TBelief cstate_;

    public:
        using state_type = TBelief;
        using action_type = TAction;
        using observation_type = TObservation;

        BeliefMDP(std::shared_ptr<DecPOMDP> underlying_pomdp);

        TBelief &getInitialState();
        TBelief &getState();
        TBelief nextState(TBelief belief, TAction action, TObservation obs) const;
        TBelief nextState(TBelief belief, TAction action) const;
        auto getActionSpace(TBelief ostate = TBelief());
        
        /**
         * @fn double getReward(Vector belief, number action);
         * @brief Get transformed reward from action and belief  
         */
        double getReward(TBelief belief, TAction action) const;

        /**
         * @brief Get the Expected Next Value object
         * 
         * @param value_function 
         * @param belief 
         * @param action 
         * @param t 
         * @return double 
         */
        double getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, TBelief belief, TAction action, int t) const;

        /**
         * @brief Get the Observation Probability p(o | b, a)
         * 
         * @param action 
         * @param obs 
         * @param belief 
         * @return double 
         */
        double getObservationProbability(TAction action, TObservation obs, TBelief belief) const;
    };
} // namespace sdm
#include <sdm/world/belief_mdp.tpp>