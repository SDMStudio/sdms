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
#include <sdm/utils/struct/graph.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/world/mdp.hpp>

namespace sdm
{
    /**
     * @brief The BaseBeliefMDP class is the interface contains the transformation of a the POMDP formalism in BaseBeliefMDP formalism.
     */
    template <class TBelief>
    class BaseBeliefMDP : public SolvableByMDP,
                          public GymInterface
    {
    public:
        BaseBeliefMDP();
        BaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp, bool store_states = true, bool store_actions = true, int batch_size = 0);

        /**
         * @brief Get the next belief.
         * This function returns the next belief. To do so, we check in the MDP graph the existance of an edge (action / observation) starting from the current belief. 
         * If exists, we return the associated next belief. Otherwise, we compute the next belief using  "computeNextStateAndProba" function and add the edge from the current belief to the next belief in the graph.
         * 
         * @param belief the belief
         * @param action the action
         * @param observation the observation
         * @param t the timestep
         * @return the next belief
         */
        std::shared_ptr<State> nextBelief(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
        
        /** @brief Get the Observation Probability p(o | b', a) */
        double getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_belief, const std::shared_ptr<Observation> &obs, number t = 0) const;

        /** @brief Get the address of the underlying POMDP */
        std::shared_ptr<POMDPInterface> getUnderlyingPOMDP() const;

        /**
         * @brief Select the next belief.
         * 
         * @param belief the current belief
         * @param action the action
         * @param t the timestep
         * @param hsvi a pointer on the algorithm that makes the call
         * @return the next state
         */
        std::shared_ptr<State> nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr);

        /**
         * @brief Get the action space at a specific belief and timestep.
         * The time dependency is required in extensive-form games in which some agents have a different action space.   
         * 
         * @param belief the belief
         * @param t the timestep
         * @return the action space 
         */
        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &belief, number t = 0);

        /**
         * @brief Get the expected reward of executing a specific action in a specific belief at timestep t. 
         * The time dependency can be required in non-stationnary problems.   
         * 
         * @param belief the belief
         * @param action the action
         * @param t the timestep
         * @return the reward
         */
        double getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Get the expected next value
         * 
         * @param value_function a pointer on the value function to use to perform the calculus.
         * @param state the state on which to evaluate the next expected value *
         * @param action 
         * @param t 
         * @return double 
         */
        double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0);

        std::shared_ptr<Observation> reset();
        std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &observation, number t);
        std::shared_ptr<Action> getRandomAction(const std::shared_ptr<Observation> &observation, number t);


        /**
         * @brief Get the graph of 
         * 
         * @return std::shared_ptr<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>> 
         */
        std::shared_ptr<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>> getMDPGraph();
        std::vector<std::shared_ptr<State>> getStoredStates() const;

        /** @brief A pointer on the bag containing all states. */
        RecursiveMap<TBelief, std::shared_ptr<State>> state_space_;
        
    protected:
        // If 0, it means the exact transitions will be used and not sampled ones.
        int batch_size_;

        /** @brief The current state (used in RL). */
        std::shared_ptr<State> current_state_;

        /** @brief The current timestep (used in RL). */
        int step_;

        /** @brief Hyperparameters. */
        bool store_states_ = true, store_actions_ = true;

        /** @brief The probability transition. (i.e. p(o | b, a) */
        RecursiveMap<std::shared_ptr<State>, std::shared_ptr<Action>, std::shared_ptr<Observation>, double> transition_probability;

        /** @brief the MDP Graph (graph of state transition) */
        std::shared_ptr<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>> mdp_graph_;

        std::shared_ptr<Graph<double, Pair<std::shared_ptr<State>, std::shared_ptr<Action>>>> reward_graph_;
        
        /**
         * @brief Compute the state transition in order to return next state and associated probability.
         * This function can be modify in an inherited class to define a belief MDP with a different representation of the belief state. 
         * (i.e. OccupancyMDP inherit from BaseBeliefMDP with TBelief = OccupancyState)
         * 
         * @param belief the belief
         * @param action the action
         * @param observation the observation
         * @param t the timestep
         * @return the couple (next state, transition probability in the next state)
         */
        virtual Pair<std::shared_ptr<State>, double> computeNextStateAndProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);

        
        virtual std::shared_ptr<State> computeNextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
        virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeExactNextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
        virtual Pair<std::shared_ptr<State>, std::shared_ptr<State>> computeSampledNextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
    };

    using BeliefMDP = BaseBeliefMDP<Belief>;

}
#include <sdm/world/belief_mdp.tpp>
