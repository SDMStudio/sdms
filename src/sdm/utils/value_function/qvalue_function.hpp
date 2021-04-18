/**
 * @file value_function.hpp
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

#include <sdm/core/state/history.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/value_function/base_value_function.hpp>
#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/world/gym_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @class QValueFunction
     * @brief This class is the abstract class of value function. All value function must derived this class.
     * 
     * @tparam TState Type of the state.
     * @tparam TAction Type of the action.
     * @tparam TValue Type of the value.
     */
    template <typename TState, typename TAction, typename TValue = double>
    class QValueFunction : public BaseValueFunction<TState, TAction, TValue>
    {
    protected:
        /**
         * @brief Initialization function. If defined, algorithms on value functions will get inital values using this function.
         * 
         */
        // std::shared_ptr<BinaryFunction<TState, TAction, number, TValue>> init_function_ = nullptr;

    public:
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
        virtual void initialize(TValue v, number t = 0) = 0;

        /**
         * @brief Get the value at a given state
         */
        TValue getValueAt(const TState &state, number t = 0);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        virtual std::shared_ptr<VectorImpl<TAction, TValue>> getQValueAt(const TState &state, number t) = 0;

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        virtual TValue getQValueAt(const TState &state, const TAction &action, number t) = 0;

        TAction getBestAction(const TState &state, number t = 0);

        TAction getBestAction_his(std::shared_ptr<GymInterface<TState, TAction>> env, const TState &state, number t = 0);

        /**
         * @brief Update the value at a given state
         */
        virtual void updateQValueAt(const TState &state, const TAction &action, number t = 0) = 0;

        /**
         * @brief Update the value at a given state (given a target)
         */
        virtual void updateQValueAt(const TState &state, const TAction &action, number t, TValue target) = 0;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() = 0;
    };
} // namespace sdm
#include <sdm/utils/value_function/qvalue_function.tpp>