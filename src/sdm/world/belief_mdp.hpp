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
#include <sdm/core/state/beliefs.hpp>
#include <sdm/world/base/base_belief_mdp.hpp>

namespace sdm
{
    /**
     * @brief The BeliefMDP class is the interface that enables solving Discret POMDP using HSVI algorithm.
     * 
     * @tparam TBelief the belief type
     * @tparam TAction the action type
     */
    template <typename TBelief = BeliefState<number>, typename TAction = number, typename TObservation = number>
    class BeliefMDP : public BaseBeliefMDP<TBelief, TAction, TObservation>
    {
    public:
        using state_type = TBelief;
        using action_type = TAction;
        using observation_type = TBelief;

        BeliefMDP();
        BeliefMDP(std::string underlying_pomdp);
        BeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp);

        TBelief nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const;
        TBelief nextState(const TBelief &belief, const TAction &action, number t, std::shared_ptr<HSVI<TBelief, TAction>> hsvi) const;

        double getReward(const TBelief &belief, const TAction &action) const;
        double getObservationProbability(const TBelief &, const TAction &action, const TObservation &obs, const TBelief &belief) const;
    };

    // ##############################################################################################################################
    // ############################ SPECIALISATION FOR BeliefState Structure ###############################################
    // ##############################################################################################################################

    /**
     * @brief Specialisation of occupancy mdp in the case of occupancy states. 
     * 
     * @tparam TActionDescriptor the action type
     * @tparam TObservation the observation type
     * @tparam TActionPrescriptor the action type (controller's one)
     */
    template <typename TAction, typename TObservation>
    class BeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>
        : public BaseBeliefMDP<BeliefStateGraph_p<TAction, TObservation>, TAction, TObservation>
    {
    public:
        using state_type = BeliefStateGraph_p<TAction, TObservation>;
        using action_type = TAction;
        using observation_type = TObservation;

        BeliefMDP();
        BeliefMDP(std::string underlying_pomdp);
        BeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp);

        state_type nextState(const state_type &belief, const TAction &action, const TObservation &obs) const;
        state_type nextState(const state_type &belief, const TAction &action, number t, std::shared_ptr<HSVI<state_type, TAction>> hsvi) const;

        double getReward(const state_type &belief, const TAction &action) const;
        double getObservationProbability(const state_type &, const TAction &action, const TObservation &obs, const state_type &belief) const;
    };
} // namespace sdm
#include <sdm/world/belief_mdp.tpp>