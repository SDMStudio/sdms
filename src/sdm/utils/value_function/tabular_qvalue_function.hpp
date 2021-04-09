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
     * @tparam TState Type of the states 
     * @tparam TAction Type of the states
     * @tparam TValue Type of the values (must be primitive type)
     * @tparam TStruct Type of vector container (MappedVector, DenseVector and SparseVector are common type) 
     */
    template <typename TState,
              typename TAction,
              typename TValue = double,
              template <typename TS, typename TA, typename TV> class TMatrix = MappedMatrix>
    class TabularQValueFunction : public QValueFunction<TState, TAction, TValue>
    {
    protected:
        using Container = TMatrix<TState, TAction, TValue>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorImpl interface can be used.
         */
        std::vector<Container> representation;

        double learning_rate_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer<TState, TAction>> initializer_;

    public:
        TabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<TState, TAction>> initializer);

        TabularQValueFunction(number horizon = 0, double learning_rate = 0.1, TValue default_value = 0.);

        /**
         * @brief Initialize the value function 
         */
        void initialize();

        /**
         * @brief Initialize the value function with a default value
         */
        void initialize(TValue v, number t = 0);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorImpl<TAction, TValue>> getQValueAt(const TState &state, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        TValue getQValueAt(const TState &state, const TAction &action, number t);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const TState &state, const TAction &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a target)
         */
        void updateQValueAt(const TState &state, const TAction &action, number t, TValue target);

        /**
         * @brief Define this function in order to be able to display the value function
         */
        std::string str();

        friend std::ostream &operator<<(std::ostream &os, TabularQValueFunction<TState, TAction> &vf)
        {
            os << vf.str();
            return os;
        }
    };

    template <typename TState, typename TAction, typename TValue = double>
    using MappedQValueFunction = TabularQValueFunction<TState, TAction, TValue, MappedMatrix>;

    // template <typename TState, typename TAction, typename TValue = double>
    // using SparseValueFunction = TabularQValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, SparseVector>;

    // template <typename TState, typename TAction, typename TValue = double>
    // using DenseValueFunction = TabularQValueFunction<TState, TAction, TValue, ClassicBellmanBackupOperator, DenseVector>;

} // namespace sdm
#include <sdm/utils/value_function/tabular_qvalue_function.tpp>
