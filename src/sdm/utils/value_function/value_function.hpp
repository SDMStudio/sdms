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

#include <sdm/core/function.hpp>
#include <sdm/utils/linear_algebra/vector_impl.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <typename TState, typename TAction>
    class SolvableByHSVI;

    /**
     * @class ValueFunction
     * @brief This class is the abstract class of value function. All value function must derived this class.
     * 
     * @tparam TState Type of the state.
     * @tparam TAction Type of the action.
     * @tparam TValue Type of the value.
     */
    template <typename TState, typename TAction, typename TValue = double>
    class ValueFunction : public Function<TState, TValue>
    {
    protected:
        /**
         * @brief The problem which incremental value function is evaluated 
         * 
         */
        std::shared_ptr<SolvableByHSVI<TState, TAction>> problem_;

        /**
         * @brief The horizon for planning.
         */
        int horizon_;

    public:
        /**
         * @brief Construct a new Incremental Value Function object
         * 
         * @param problem 
         * @param default_value 
         */
        ValueFunction(std::shared_ptr<SolvableByHSVI<TState, TAction>> problem, int horizon);

        /**
         * @brief Initialize the value function 
         */
        virtual void initialize() = 0;

        /**
         * @brief Initialize the value function with a default value
         */
        virtual void initialize(TValue v, int t = 0) = 0;

        /**
         * @brief Get the value at a given state
         */
        virtual TValue getValueAt(const TState &state, int t = 0) = 0;

        /**
         * @brief Update the value at a given state
         */
        virtual void updateValueAt(const TState &s, int t = 0) = 0;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() = 0;

        TValue operator()(const TState &state);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorImpl<TAction, TValue>> getQValueAt(const TState &state, int t = 0);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        TValue getQValueAt(const TState &state, const TAction &action, int t = 0);

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        TAction getBestAction(const TState &state, int t = 0);

        std::shared_ptr<SolvableByHSVI<TState, TAction>> getWorld();

        int getHorizon() const;

        bool isFiniteHorizon() const;

        bool isInfiniteHorizon() const;

        friend std::ostream &operator<<(std::ostream &os, ValueFunction<TState, TAction> &vf)
        {
            os << vf.str();
            return os;
        }

        double getDiscount(int t);
    };
} // namespace sdm
#include <sdm/utils/value_function/value_function.tpp>