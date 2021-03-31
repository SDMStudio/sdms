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
#include <sdm/core/state/state.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_mmdp.hpp>

namespace sdm
{
    /**
     * @brief The BeliefMDP class is the interface that enables solving Discret POMDP using HSVI algorithm.
     * 
     * @tparam TBelief the belief type
     * @tparam TAction the action type
     */
    template <typename TBelief = BeliefState, typename TAction = number, typename TObservation = number>
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

        BeliefMDP();
        BeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp);
        BeliefMDP(std::string underlying_pomdp);

        TBelief reset();
        TBelief &getState();

        bool isSerialized() const;
        DiscretePOMDP *getUnderlyingProblem();

        TBelief getInitialState();
        TBelief nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const;
        TBelief nextState(const TBelief &belief, const TAction &action, int t = 0, HSVI<TBelief, TAction> *hsvi = nullptr) const;

        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TBelief &ostate = TBelief());

        double getReward(const TBelief &belief, const TAction &action) const;
        double getExpectedNextValue(ValueFunction<TBelief, TAction> *value_function, const TBelief &belief, const TAction &action, int t) const;

        /**
         * @brief Get the Observation Probability p(o | b, a)
         */
        double getObservationProbability(const TAction &action, const TObservation &obs, const TBelief &belief) const;

/**
         * @brief Get the corresponding Markov Decision Process. It corresponds to the reformulation of the Belief MDP in a MDP where the blief state space is the state space. 
         * 
         * @return a belief MDP
         */
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. In this particular case, it will return the current MDP
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();

    };
} // namespace sdm
#include <sdm/world/belief_mdp.tpp>