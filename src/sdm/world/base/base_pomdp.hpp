/**
 * @file discrete_pomdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the BasePOMDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/world/base/base_mdp.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Partially Observable Markov Decision Processes. 
     */
    class BasePOMDP : public BaseMDP, public std::enable_shared_from_this<BasePOMDP>
    {
    public:
        /**
         * @brief Get the observation probability (i.e. p_t(o | a, x'))
         * 
         * @param action the action
         * @param next_state the next state
         * @param observation the observation
         * @param t the timestep
         * @return double the probability 
         */
        double getObservationProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t = 0) const;
        
        /**
         * @brief Get the dynamics (i.e. p_t(x', o | x,a))
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param observation 
         * @return double 
         */
        double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation) const;

        std::shared_ptr<BasePOMDP> getptr();

        /**
         * @brief Get the corresponding Markov Decision Process. It corresponds to the relaxation of the original POMP assuming that the agent can observation the state of the environment. 
         * 
         * @return a MDP 
         */
        std::shared_ptr<BaseMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. It corresponds to the reformulation of the original POMP in a MDP where the state space is the space of beliefs. 
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP> toBeliefMDP();
    };
} // namespace sdm
