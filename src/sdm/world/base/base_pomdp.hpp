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
    class BasePOMDP : public BaseMDP, virtual public POMDPInterface
    {
    public:
        BasePOMDP(number num_agents,
                  double discount,
                  const std::shared_ptr<Space<std::shared_ptr<State>>> &state_space,
                  const std::shared_ptr<Space<std::shared_ptr<Action>>> &action_space,
                  const std::shared_ptr<Space<std::shared_ptr<Observation>>> &obs_space,
                  const std::shared_ptr<BaseReward> &reward,
                  const std::shared_ptr<BaseStateDynamics> &state_dynamics,
                  const std::shared_ptr<BaseObservationDynamics> &obs_dynamics);

        /**
         * @brief Get the reachable next states
         * 
         * @param state the state
         * @param action the action
         * @return the set of reachable states
         */
        std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const;

        std::set<std::shared_ptr<Observation>> getAllObservations(number t) const;

        /**
         * @brief Get the observation probability (i.e. p_t(o | a, x'))
         * 
         * @param action the action
         * @param next_state the next state
         * @param observation the observation
         * @param t the timestep
         * @return double the probability 
         */
        double getObservationProbability(const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

        /**
         * @brief Get the dynamics (i.e. p_t(x', o | x,a))
         * 
         * @param state 
         * @param action 
         * @param next_state 
         * @param observation 
         * @return double 
         */
        double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const;

    protected:
        std::shared_ptr<Space<std::shared_ptr<Observation>>> obs_space_;
        std::shared_ptr<BaseObservationDynamics> obs_dynamics_;
    };
} // namespace sdm
