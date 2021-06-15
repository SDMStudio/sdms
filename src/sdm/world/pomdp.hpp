/**
 * @file discrete_pomdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the POMDP class.
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
#include <sdm/core/dynamics/tabular_observation_dynamics_SAS.hpp>

#include <sdm/world/mdp.hpp>
#include <sdm/world/base/pomdp_interface.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Partially Observable Markov Decision Processes. 
     */
    class POMDP : virtual public MDP, virtual public POMDPInterface
    {
    public:
        POMDP(const std::shared_ptr<Space> &state_space,
              const std::shared_ptr<Space> &action_space,
              const std::shared_ptr<Space> &obs_space,
              const std::shared_ptr<RewardInterface> &reward,
              const std::shared_ptr<TabularStateDynamics> &state_dynamics,
              const std::shared_ptr<TabularObservationDynamicsSAS> &obs_dynamics,
              const std::shared_ptr<Distribution<std::shared_ptr<State>>> &start_distrib,
              number horizon = 0,
              double discount = 0.99,
              Criterion criterion = Criterion::REW_MAX);

        /**
         * @brief Get ths observation space at timestep t.
         * 
         * @param t the timestep
         * @return the observation space
         */
        virtual std::shared_ptr<Space> getObservationSpace(number t = 0) const;

        /**
         * @brief Get reachable observations
         * 
         * @param state the current state
         * @param action the current action
         * @param next_state the next state
         * @return the set of reachable observations
         */
        virtual std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const;

        /**
         * @brief Get the observation probability, i.e. p(o | s', a)
         * 
         * @param action the action
         * @param next_state the next state
         * @param observation the observation
         * @param t the timestep
         * @return the probability
         */
        virtual double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t = 0) const;

        /**
         * @brief Get the dynamics, i.e. p(s', o | s, a)
         * 
         * @param state the state at timestep t
         * @param action the action 
         * @param next_state the next state, i.e. timestep t+1
         * @param observation the observation
         * @param t the timestep
         * @return the probability
         */
        virtual double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t = 0) const;

        std::shared_ptr<TabularObservationDynamicsSAS> getObservationDynamics() const;

        std::shared_ptr<Observation> sampleNextObservation(const std::shared_ptr<State>& state, const std::shared_ptr<Action>& action);

    protected:
        std::shared_ptr<Space> obs_space_;
        std::shared_ptr<TabularObservationDynamicsSAS> obs_dynamics_;
    };
} // namespace sdm
