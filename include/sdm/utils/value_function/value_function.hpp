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

#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/world/posg.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @class ValueFunction
     * @brief This class is the abstract class of value function. All value function must derived this class.
     * 
     * @tparam TState Type of the state.
     * @tparam TAction Type of the action.
     * @tparam TValue Type of the value.
     */
    template <typename TState, typename TAction, typename TValue = double>
    class ValueFunction
    {
    protected:
        /**
         * @brief The problem which incremental value function is evaluated 
         * 
         */
        std::shared_ptr<POSG> problem_;

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
        ValueFunction(std::shared_ptr<POSG> problem, int horizon) : problem_(problem), horizon_(horizon)
        {
        }

        virtual void updateValueAt(const TState &s, int t = 0) = 0;

        /**
         * @brief Initialize the value function 
         * 
         */
        virtual void initialize() = 0;

        /**
         * @brief Initialize the value function 
         * 
         */
        virtual void initialize(TValue v, int t = 0) = 0;

        /**
         * @brief Get the value of the function on one point
         * 
         * @param state The point where we want the value
         * @return The value of the bound on that point 
         */
        virtual TValue getValueAt(const TState &state, int t = 0) = 0;

        /**
         * @brief Get the q value on a state 
         * 
         * @param state The state where we want to evaluate q-value
         * @return The Q Value at this state
         */
        virtual std::shared_ptr<VectorImpl<TAction, TValue>> getQValueAt(const TState &state, int t = 0) = 0;

        /**
         * @brief Get the q value on one couple (state, action) 
         * 
         * @param state The state where we want the value
         * @param action The action where we want the value
         * @return The Q Value 
         */
        virtual TValue getQValueAt(const TState &state, const TAction &action, int t = 0) = 0;

        /**
         * @brief Get the next action to do
         * 
         * @param state The point where we want the best action
         * @return The next action
         */
        virtual TAction getBestAction(const TState &state, int t = 0) = 0;

        virtual std::string str() = 0;

        std::shared_ptr<POSG> getWorld()
        {
            return this->problem_;
        }

        int getHorizon() const
        {
            return this->horizon_;
        }

        int isFiniteHorizon() const
        {
            return (this->horizon_ > 0);
        }

        int isInfiniteHorizon() const
        {
            return !(this->isFiniteHorizon());
        }
    };
} // namespace sdm