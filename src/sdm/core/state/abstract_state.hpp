/**
 * @file sequential_occupancy_state.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for sequential occupancy state class
 * @version 0.1
 * @date 14/04/2022
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/item.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/core/observation/observation.hpp>
#include <sdm/utils/value_function.hpp>

namespace sdm
{
    /**
     * @brief A public interface for states.
     *
     * Any class inheriting from this interface will be considered as generic state for algorithms.
     * Consider sections [ Theoritical Background](/tutorials/theory.html) and [ Algorithms](/tutorials/algorithms/) for more information.
     *
     */
    class AbstractState
    {
    public:
        /**
         * @brief Compute the following state given an action a.
         *
         * @param mdp the Markov Decision Process,  TO be delated later on, currently kept for the consistency with interface State.
         * @param action the action
         * @param t the timestep
         * @return std::shared_ptr<State> the next state
         */
        virtual Pair<std::shared_ptr<State>, double> next(const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t);

        /**
         * @brief Compute the reward of being in the current state and execute action a.
         *
         * @param mdp the Markov Decision Process,  TO be delated later on, currently kept for the consistency with interface State.
         * @param action the action
         * @param t the timestep
         * @return double the reward
         */
        virtual double getReward(const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Get the hash of the state.
         *
         * The hash is used in tabular value functions in order to compare efficiently two states.
         * This function must be reimplemented in inherited classes.
         *
         * @return size_t the hash code
         */
        virtual size_t hash(double precision = -1) const;

        /**
         * @brief Check equality between two states.
         *
         * This function must be implemented in inherited classes.
         *
         * @param other the state to be compared to current state
         * @return true if states are equal
         * @return false if they are different
         */
        virtual bool isEqual(const std::shared_ptr<State> &other, double precision = -1) const;

        /**
         * @brief Get string representation of the state.
         */
        virtual std::string str() const = 0;

        virtual std::pair<std::shared_ptr<Action>, double> bellman_operator(TabularValueFunction *value_fn, int t) {}
        virtual std::pair<std::shared_ptr<Action>, double> bellman_operator(PWLCValueFunction *value_fn, int t) {}
        virtual std::pair<std::shared_ptr<Action>, double> bellman_operator(TabularQValueFunction *qvalue_fn, int t) {}
        virtual std::pair<std::shared_ptr<Action>, double> bellman_operator(PWLCQValueFunction *qvalue_fn, int t) {}

        virtual std::shared_ptr<Action> greedy_operator(TabularValueFunction *value_fn, int t)
        {
            return this->bellman_operator(value_fn, t).first;
        }

        virtual std::shared_ptr<Action> greedy_operator(PWLCValueFunction *value_fn, int t)
        {
            return this->bellman_operator(value_fn, t).first;
        }

        virtual std::shared_ptr<Action> greedy_operator(TabularQValueFunction *value_fn, int t)
        {
            return this->bellman_operator(value_fn, t).first;
        }

        virtual std::shared_ptr<Action> greedy_operator(PWLCQValueFunction *value_fn, int t)
        {
            return this->bellman_operator(value_fn, t).first;
        }

        /**
         * @brief Compute the dot product operation with an alpha vector.
         *
         * @param alpha the alpha vector
         * @return double the resulting product
         */
        virtual double product(const std::shared_ptr<AlphaVector> &alpha);

        /**
         * @brief Compute the dot product operation with a beta vector.
         *
         * @param beta the beta vector
         * @param action the action
         * @return double the resulting product
         */
        virtual double product(const std::shared_ptr<BetaVector> &beta, const std::shared_ptr<Action> &action);

    private:
        std::shared_ptr<ActionSpace> computeActionSpaceAt(number t);
    };
} // namespace sdm