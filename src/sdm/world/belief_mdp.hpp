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
#include <sdm/core/action/action.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/mdp.hpp>

namespace sdm
{
    /**
     * @brief The BeliefMDP class is the interface that enables solving Discret POMDP using HSVI algorithm.
     * 
     * @tparam std::shared_ptr<State> the belief type
     * @tparam std::shared_ptr<Action> the action type
     */
    class BeliefMDP : public MDP
    {
    public:
        BeliefMDP();

        BeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp);

        // std::shared_ptr<State> reset();

        // std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

        std::shared_ptr<State> getInitialState();

        virtual std::shared_ptr<State> nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &obs) const = 0;

        virtual std::shared_ptr<State> nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t, std::shared_ptr<HSVI> hsvi) const = 0;

        std::shared_ptr<DiscreteSpace<std::shared_ptr<Action>>> getActionSpaceAt(const std::shared_ptr<State> &ostate = std::shared_ptr<State>());

        virtual double getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const = 0;

        double getExpectedNextValue(std::shared_ptr<ValueFunction> value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t) const;

        /**
         * @brief Get the Observation Probability p(o | b, a)
         */
        virtual double getObservationProbability(const std::shared_ptr<State> &, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &obs, const std::shared_ptr<State> &belief) const = 0;

    protected:
        std::shared_ptr<BeliefInterface> initial_state_;
        std::shared_ptr<BeliefInterface> current_state_;

        std::shared_ptr<POMDPInterface> getUnderlyingPOMDP();

    };

}