/**
 * @file value_function.hpp
 * @author Jérôme ARJONILLA 
 * @brief Defines the Tabular Qvalue function conditioning interface.
 * @version 0.1
 * @date 17/08/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/utils/value_function/qvalue_function_conditioning.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <class TCondition, class TState>
    class TabularQValueFunctionConditioning : public TabularQValueFunction<Pair<TCondition,TState>>, public QValueFunctionConditioning<TCondition,TState>
    {

    public:
        using TInput = typename QValueFunctionConditioning<TCondition,TState>::TInput;
        using Container = typename TabularQValueFunction<Pair<TCondition,TState>>::Container;

        TabularQValueFunctionConditioning(number horizon, double learning_rate, std::shared_ptr<QInitializer<TInput>> initializer, bool active_learning = true);

        TabularQValueFunctionConditioning(number horizon = 0, double learning_rate = 0.1, double default_value = 0., bool active_learning = true);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const TInput &state, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t, double delta);


        // Fonction conditioning

        double getQValueAt(const TCondition&, const TState &state, const std::shared_ptr<Action> &action, number t);

        void updateQValueAt(const TCondition&, const TState &state, const std::shared_ptr<Action> &action, number t = 0);

        void updateQValueAt(const TCondition&, const TState &state, const std::shared_ptr<Action> &action, number t, double delta);

    };
} // namespace sdm
#include <sdm/utils/value_function/tabular_qvalue_function_conditioning.tpp>
