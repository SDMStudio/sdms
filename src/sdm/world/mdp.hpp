/**
 * @file mdp.hpp
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
#include <sdm/core/reward/reward_interface.hpp>
#include <sdm/core/dynamics/state_dynamics_interface.hpp>
#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes. 
     * 
     */
    class MDP : virtual public MDPInterface
    {
    public:
        MDP(const std::shared_ptr<Space> &state_space,
            const std::shared_ptr<Space> &action_space,
            const std::shared_ptr<RewardInterface> &reward,
            const std::shared_ptr<StateDynamicsInterface> &state_dynamics,
            const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
            number horizon = 0,
            double discount = 0.99,
            Criterion criterion = Criterion::REW_MAX);

        virtual ~MDP();

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
         * @brief Get the number of agents
         * 
         * @return the number of agents
         */
        number getHorizon() const;

        /**
         * @brief Get the initial distribution over states.
         * 
         * @return the initial distribution over states
         */
        std::shared_ptr<Distribution<std::shared_ptr<State>>> getStartDistribution() const;

        /**
         * @brief Get ths state space at timestep t.
         * 
         * @param t the timestep
         * @return the state space
         */
        std::shared_ptr<Space> getStateSpace(number t = 0) const;

        /**
         * @brief Get ths action space at timestep t.
         * 
         * @param t the timestep
         * @return the action space
         */
        std::shared_ptr<Space> getActionSpace(number t = 0) const;

        /**
         * @brief Get the reward of executing action a in state s at timestep t.
         * 
         * @param state the state
         * @param action the action
         * @param t the timestep
         * @return the value of the reward
         */
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the reward function
         * 
         * @return the reward function
         */
        std::shared_ptr<RewardInterface> getReward() const;

        double getMinReward(number t = 0) const;

        double getMaxReward(number t = 0) const;

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

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        std::set<std::shared_ptr<State>> getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the state dynamics
         * 
         * @return the state dynamics interface
         */
        std::shared_ptr<StateDynamicsInterface> getStateDynamics() const;

    protected:
        number num_agents_, horizon_;

        double discount_;

        Criterion criterion_;

        std::shared_ptr<Space> state_space_;

        std::shared_ptr<Space> action_space_;

        std::shared_ptr<RewardInterface> reward_;

        std::shared_ptr<StateDynamicsInterface> state_dynamics_;

        std::shared_ptr<Distribution<std::shared_ptr<State>>> start_distrib_;
    };

} // namespace sdm