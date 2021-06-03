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

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/distribution.hpp>
#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{
    class SerializedMMDP : public MDPInterface
    {
    public:
        SerializedMMDP(const std::shared_ptr<MDPInterface> &mmdp);

        /**
         * @brief Get the identifier of the current agent.
         * 
         * @param t the timestep
         * @return number the agent id
         */
        number getAgentId(number t);

        /**
         * @brief 
         * 
         * @param t 
         * @return true 
         * @return false 
         */
        bool isLastAgent(number t);

        /**
         * @brief Get the discount factor at timestep t.
         * 
         * @param t the timestep
         * @return the discount factor
         */
        virtual double getDiscount(number t) const;

        /**
         * @brief Get the initial distribution over states.
         * 
         * @return the initial distribution over states
         */
        virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const;

        /**
         * @brief Get all states
         * 
         * @return the set of states 
         */
        virtual std::set<std::shared_ptr<State>> getAllStates(number t) const;

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        virtual std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;

        /**
         * @brief Get all actions
         * 
         * @return the set of actions 
         */
        virtual std::set<std::shared_ptr<Action>> getAllActions(number t) const;

        /**
         * @brief Get the reward
         * 
         * @param state 
         * @param action 
         * @param t 
         * @return double 
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;

        virtual double getMinReward(number t) const;
        
        virtual double getMaxReward(number t) const;
        /**
         * @brief Get the Transition Probability object
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param t 
         * @return double 
         */
        virtual double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const;

    protected:
        std::shared_ptr<MDPInterface> mmdp_;
    };

} // namespace sdm