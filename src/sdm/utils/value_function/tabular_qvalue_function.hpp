/**
 * @file tabular_value_function.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Tabular value function are functions of state and action that use a vector representation. 
 * @version 0.1
 * @date 16/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <iostream>
#include <type_traits>

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/backup_operator/backup_operator.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief Tabular value function are functions of state and action that use a vector representation to store the values. 
     * 
     * @tparam TMatrix Type of matrix container (MappedMatrix, DenseMatrix and SparseMatrix are common type) 
     */
    template <template <typename TS, typename TA, typename TV> class TMatrix = MappedMatrix>
    class TabularQValueFunction : public QValueFunction
    {
    protected:
        using Container = TMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<Container> representation;

        double learning_rate_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer> initializer_;

    public:
        TabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer);

        TabularQValueFunction(number horizon = 0, double learning_rate = 0.1, double default_value = 0.);

        /**
         * @brief Initialize the value function 
         */
        void initialize();

        /**
         * @brief Initialize the value function with a default value
         */
        void initialize(double v, number t = 0);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValueAt(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a target)
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double target);

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, TabularQValueFunction<std::shared_ptr<State>, std::shared_ptr<Action>> &vf)
        {
            os << vf.str();
            return os;
        }
    };

    template <typename std::shared_ptr<State>, typename std::shared_ptr<Action>, typename double = double>
    using MappedQValueFunction = TabularQValueFunction<std::shared_ptr<State>, std::shared_ptr<Action>, double, MappedMatrix>;

    // template <typename std::shared_ptr<State>, typename std::shared_ptr<Action>, typename double = double>
    // using SparseValueFunction = TabularQValueFunction<std::shared_ptr<State>, std::shared_ptr<Action>, double, ClassicBellmanBackupOperator, SparseVector>;

    // template <typename std::shared_ptr<State>, typename std::shared_ptr<Action>, typename double = double>
    // using DenseValueFunction = TabularQValueFunction<std::shared_ptr<State>, std::shared_ptr<Action>, double, ClassicBellmanBackupOperator, DenseVector>;

} // namespace sdm