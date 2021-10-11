/**
 * @file qvalue_function.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Defines the value function interface.
 * @version 0.1
 * @date 16/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <memory>

#include <sdm/core/function.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>
#include <sdm/utils/value_function/base_value_function.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @class QValueFunction
     * 
     * @brief This abstract class is made up of various methods specific to Q-value functions. 
     * 
     * All Q-value functions must inherit from this class to be considered as data structures 
     * usable by reinforcement learning algorithms
     * 
     */
    class QValueFunction : public QValueFunctionBase
    {
    protected:
        /**
         * @brief Initialization function. If defined, algorithms on value functions will get inital values using this function.
         * 
         */
        // std::shared_ptr<BinaryFunction<std::shared_ptr<State>, std::shared_ptr<Action>, number, double>> init_function_ = nullptr;

    public:
        using TGlobalInput = typename QValueFunctionBase::T;

        QValueFunction();

        /**
         * @brief Construct a new Incremental Value Function object
         * 
         * @param problem 
         * @param default_value 
         */
        QValueFunction(number horizon);

        /**
         * @brief Initialize the value function 
         */
        virtual void initialize() = 0;

        /**
         * @brief Initialize the value function with a default value
         */
        virtual void initialize(double v, number t = 0) = 0;

        /**
         * @brief Get the value at a given state
         */
        double getValueAt(const TGlobalInput &input, number t = 0);

        /**
         * @brief Update the value at a given state
         */
        void updateValueAt(const TGlobalInput &input, number t = 0);

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        std::shared_ptr<Action> getBestAction(const TGlobalInput &input, number t);
        
        /**
         * @brief Get the q-values for all actions at a state
         * 
         * @param state the state
         * @return the q-value vector 
         */
        virtual std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const std::shared_ptr<State> &state, number t) = 0;

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        virtual double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) = 0;

        /**
         * @brief Update the value at a given state
         */
        virtual void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) = 0;

        /**
         * @brief Update the value at a given state (given a target)
         */
        virtual void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double target, number t = 0) = 0;

        /**
         * @brief Get the number of accessible states
         */
        virtual int getNumStates() const = 0;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const = 0;

        /**
         * @brief Get shared pointer on the current QValueFunction
         */
        std::shared_ptr<QValueFunction> getptr();

    };
} // namespace sdm
