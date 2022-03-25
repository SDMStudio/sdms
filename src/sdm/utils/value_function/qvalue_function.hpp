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

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    using namespace update;

    /**
     * @class QValueFunction
     *
     * @brief This abstract class is made up of various methods specific to Q-value functions.
     *
     * All Q-value functions must inherit from this class to be considered as data structures
     * usable by reinforcement learning algorithms
     *
     */
    class QValueFunction : virtual public ValueFunctionInterface
    {

    public:
        QValueFunction();

        QValueFunction(const std::shared_ptr<SolvableByDP> &world,
                       const std::shared_ptr<Initializer> &initializer = nullptr,
                       const std::shared_ptr<ActionSelectionInterface> &action = nullptr,
                       const std::shared_ptr<QUpdateOperatorInterface> &update_operator = nullptr);

        /**
         * @brief Get the value at a given state
         */
        double getValueAt(const std::shared_ptr<State> &state, number t = 0);

        /**
         * @brief Get the q-value given state and action
         *
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        virtual double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) = 0;

        /**
         * @brief Update the value at a given time step
         */
        void updateValueAt(double learning_rate, number t = 0);

        /**
         * @brief Get the update operator
         */
        std::shared_ptr<QUpdateOperatorInterface> getUpdateOperator() const;

        /**
         * @brief Set the update operator
         * 
         * @param update_operator the update operator
         */
        void setUpdateOperator(std::shared_ptr<QUpdateOperatorInterface> update_operator);

        /**
         * @brief Get a shared pointer on this q-value function
         */
        std::shared_ptr<QValueFunction> getptr();

    protected:
        /**
         * @brief The operator used to update the value function
         */
        std::shared_ptr<QUpdateOperatorInterface> update_operator_;
    };
} // namespace sdm
