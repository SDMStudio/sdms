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
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/base/pomdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The BeliefMDP class is the interface contains the transformation of a the POMDP formalism in BeliefMDP formalism.
     */
    class BeliefMDP : public SolvableByMDP
    {
    public:
        BeliefMDP();
        BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp);

        virtual std::shared_ptr<State> nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr) const;

        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &belief, number t = 0);

        virtual double getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0) const;

        double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the Observation Probability p(o | b', a)
         */
        virtual double getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_belief, const std::shared_ptr<Observation> &obs, number t = 0) const;


    protected:
        std::shared_ptr<State> current_state_;

        std::shared_ptr<POMDPInterface> getUnderlyingPOMDP() const;
        
        /**
         * @brief This part of the code can be used to 
         * 
         * @param belief 
         * @param action 
         * @param obs 
         * @param t 
         * @return std::shared_ptr<BeliefInterface> 
         */
        static Pair<std::shared_ptr<BeliefInterface>, double> nextBelief(const std::shared_ptr<POMDPInterface> &pomdp, const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &obs, number t = 0);
        
        std::shared_ptr<State> nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t) const;
    };

}