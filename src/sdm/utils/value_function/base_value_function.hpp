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

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector_impl.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @class BaseValueFunction
     * @brief This class is the abstract class of all kind of value functions. All {state,action,q}-value function must derived this class.
     * 
     * @tparam TState Type of the state.
     * @tparam TAction Type of the action.
     * @tparam TValue Type of the value.
     */
    template <typename TState, typename TAction, typename TValue = double>
    class BaseValueFunction
    {
    protected:
        /**
         * @brief The horizon for planning.
         */
        number horizon_;

    public:
        BaseValueFunction();

        BaseValueFunction(number horizon);

        /**
         * @brief Destroy the value function
         * 
         */
        virtual ~BaseValueFunction() {}

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
        virtual TValue getValueAt(const TState &state, number t = 0) =0;

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

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        virtual TAction getBestAction(const TState &state, number t) = 0;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() = 0;

        int getHorizon() const;

        bool isFiniteHorizon() const;

        bool isInfiniteHorizon() const;

        friend std::ostream &operator<<(std::ostream &os, BaseValueFunction<TState, TAction> &vf)
        {
            os << vf.str();
            return os;
        }
    };
} // namespace sdm
#include <sdm/utils/value_function/base_value_function.tpp>