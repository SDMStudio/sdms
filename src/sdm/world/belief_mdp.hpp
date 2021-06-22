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
     * @brief The BeliefMDP class is the interface that enables solving Discret POMDP using HSVI algorithm.
     * 
     * @tparam std::shared_ptr<State> the belief type
     * @tparam std::shared_ptr<Action> the action type
     */
    class BeliefMDP : public SolvableByMDP
    {
    public:
        BeliefMDP();

        BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp);

        BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, const Belief &initial_belief);

        // std::shared_ptr<State> reset();

        // std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

        // std::shared_ptr<State> getInitialState();

        virtual std::shared_ptr<State> nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr) const;

        std::shared_ptr<Distribution<std::shared_ptr<State>>> getNextStateDistribution(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const;

        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &belief, number t = 0);

        virtual double getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0) const;

        double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the Observation Probability p(o | b', a)
         */
        virtual double getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_belief, const std::shared_ptr<Observation> &obs, number t = 0) const;

        std::shared_ptr<POMDPInterface> getUnderlyingPOMDP() const;

    protected:
        // std::shared_ptr<State> initial_state_;
        std::shared_ptr<State> current_state_;

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

        /**
         * @brief A pointer on the bag containing all nodes.
         */
        RecursiveMap<std::shared_ptr<Action>, std::shared_ptr<Observation>, double> belief_proba;
    };

}