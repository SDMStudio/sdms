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
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
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
    class HierarchicalTabularQValueFunction : public QValueFunction<TState, TAction, TValue>
    {
    protected:
        using Container = TabularQValueFunction<typename TState::second_type, TAction, TValue>;

        /**
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorImpl interface can be used.
         */
        std::unordered_map<typename TState::first_type, Container> representation;

        double learning_rate_;
        number horizon_;
        std::shared_ptr<HierarchicalPrivateOccupancyMDP<typename TState::first_type, HierarchicalPrivateJointDeterministicDecisionRule<Joint<HistoryTree_p<number>>, number>>> env_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer<typename TState::second_type, TAction>> initializer_;

    public:
        HierarchicalTabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<typename TState::second_type, TAction>> initializer);
        HierarchicalTabularQValueFunction(
            number horizon, 
            double learning_rate, 
            std::shared_ptr<QInitializer<typename TState::second_type, TAction>> initializer, 
            std::shared_ptr<HierarchicalPrivateOccupancyMDP<typename TState::first_type, HierarchicalPrivateJointDeterministicDecisionRule<Joint<HistoryTree_p<number>>, number>>> env
        );

        HierarchicalTabularQValueFunction(number horizon = 0, double learning_rate = 0.1, TValue default_value = 0.);

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
        std::shared_ptr<VectorImpl<TAction, TValue>> getQValuesAt(const TState &state, number t);

        /**
         * @brief Get the q-value at a state
         * 
         * @param private_occupancy_state the private occupancy state of agent N
         * @param hsitory the joint history
         * @return the action value vector 
         */
        std::shared_ptr<VectorImpl<TAction, TValue>> getQValuesAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        TValue getQValueAt(const TState &state, const TAction &action, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param private_occupancy_state the private occupancy state of agent N
         * @param hsitory the joint history
         * @param action the action
         * @return the q-value
         */
        TValue getQValueAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, const TAction &action, number t);

        TAction getBestAction(const TState &state, number t = 0);

        HierarchicalPrivateJointDeterministicDecisionRule<Joint<HistoryTree_p<observation>>, action> getGreedyAction(const TState &state, number t = 0);

        /**
         * @brief Get the target q-value given state
         * 
         * @param state the state
         * @return the q-value
         */
        TValue getNextValueAt(const TState &state, number t);

        /**
         * @brief Get the target q-value given state
         * 
         * @param private_occupancy_state the private occupancy state of agent N
         * @param hsitory the joint history
         * @return the q-value
         */
        TValue getNextValueAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, number t);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const TState &state, const TAction &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const TState &state, const TAction &action, number t, TValue delta);

        void updateQValueAt(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history, const TAction &action, number t, TValue delta);

        /**
         * @brief 
         */
        void initializeIfNeeded(const TState &state);

        Joint<HistoryTree_p<number>> get_o(const typename TState::first_type &private_occupancy_state, const typename TState::second_type &history);

        /**
         * @brief Define this function in order to be able to display the value function
         */
        std::string str();

        // friend std::ostream &operator<<(std::ostream &os, TabularQValueFunction<TState, TAction> &vf)
        // {
        //     os << vf.str();
        //     return os;
        // }
    };

    template <typename TState, typename TAction, typename TValue = double>
    using HierarchicalMappedQValueFunction = HierarchicalTabularQValueFunction<TState, TAction, TValue, MappedMatrix>;

} // namespace sdm
#include <sdm/utils/value_function/hierarchical_qvalue_function.tpp>