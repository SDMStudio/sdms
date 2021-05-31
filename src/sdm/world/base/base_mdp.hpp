/**
 * @file discrete_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the MDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/parser/parser.hpp>
#include <sdm/exception.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>
#include <sdm/world/decision_process.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state_dynamics.hpp>
#include <sdm/core/reward.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class BaseMDP : public SolvableByHSVI,
                    public std::enable_shared_from_this<MDP>
    {
    public:
        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        number getNumAgents() const;

        /**
         * @brief Get the discount factor at timestep t.
         * 
         * @param t the timestep
         * @return the discount factor
         */
        double getDiscount(number t = 0) const;

        /**
         * @brief Get the initial distribution over states.
         * 
         * @return the initial distribution over states
         */
        Distribution<std::shared_ptr<State>> getStartDistribution() const;

        /**
         * @brief Get all states
         * 
         * @return the set of states 
         */
        std::set<std::shared_ptr<State>> getAllStates() const;

        /**
         * @brief Get all actions
         * 
         * @return the set of actions 
         */
        std::set<std::shared_ptr<Action>> getAllActions() const;

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const;

        /**
         * @brief Get the reward
         * 
         * @param state 
         * @param action 
         * @param t 
         * @return double 
         */
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the Transition Probability object
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param t 
         * @return double 
         */
        double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;
    };

} // namespace sdm