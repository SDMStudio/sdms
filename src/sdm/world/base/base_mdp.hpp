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
#include <sdm/core/space/space.hpp>
#include <sdm/core/reward/base_reward.hpp>
#include <sdm/core/dynamics/state_dynamics_interface.hpp>
#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class BaseMDP : public MDPInterface
    {
    public:
        BaseMDP(number num_agents,
                double discount,
                const std::shared_ptr<Space> &state_space,
                const std::shared_ptr<Space> &action_space,
                const std::shared_ptr<BaseReward> &reward,
                const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
                const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib);

        virtual ~BaseMDP();
 
        /**
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        virtual number getNumAgents() const;

        /**
         * @brief Get the discount factor at timestep t.
         * 
         * @param t the timestep
         * @return the discount factor
         */
        virtual double getDiscount(number t = 0) const;

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
        virtual std::vector<std::shared_ptr<State>> getAllStates(number t = 0) const;

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        virtual std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get all actions
         * 
         * @return the set of actions 
         */
        virtual std::vector<std::shared_ptr<Action>> getAllActions(number t = 0) const;

        /**
         * @brief Get the reward
         * 
         * @param state 
         * @param action 
         * @param t 
         * @return double 
         */
        virtual double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;
        
        virtual double getMinReward(number t = 0) const;

        virtual double getMaxReward(number t = 0) const;

        /**
         * @brief Get the Transition Probability object
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param t 
         * @return double 
         */
        virtual double getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t = 0) const;

        const std::shared_ptr<Space> &getStateSpace(number t = 0) const;

        const std::shared_ptr<Space> &getActionSpace(number t = 0) const;

    protected:
        number num_agents_ = 2;

        double discount_ = 1.0;

        std::shared_ptr<Space> state_space_;

        std::shared_ptr<Space> action_space_;

        std::shared_ptr<BaseReward> reward_;

        std::shared_ptr<StateDynamicsInterface> state_dynamics_;

        std::shared_ptr<Distribution<std::shared_ptr<State>>> start_distrib_;
    };

} // namespace sdm