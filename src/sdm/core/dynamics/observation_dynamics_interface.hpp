#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

namespace sdm
{
    class ObservationDynamicsInterface
    {
    public:
        /**
         * @brief Get reachable observations from a state, suppose a specific action was executed. 
         * 
         * @param state the current state
         * @param action the current action
         * @param next_state the state at time t +1
         * @param t the timestep
         * @return the list of next reachable observations 
         */
        virtual std::set<std::shared_ptr<Observation>> getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action,const std::shared_ptr<State> &next_state, number t) const = 0;

        /**
         * @brief Get the the state / observation transition probability (i.e. p(o | s, a, s')).
         * 
         * @param state the state
         * @param action the action
         * @param obs the observation
         * @param next_state the next state
         * @param t the timestep
         * @return double the probability
         */
        virtual double getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const = 0;

        /**
         * @brief Get the the state / observation transition probability (i.e. p(s', o | s, a)).
         * 
         * @param state the state 
         * @param action the action
         * @param next_state the next state
         * @param observation the observation
         * @param t the timestep
         * @return double the probability
         */
        virtual double getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const = 0;
    };
} // namespace sdm
