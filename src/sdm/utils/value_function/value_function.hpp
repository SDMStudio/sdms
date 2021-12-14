#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/world/solvable_by_dp.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

    using namespace update;

    /**
     * @brief This class contains attributes and methods common to all value functions.
     *
     * Some attributes are callable. They will be called to update the value function (i.e. the initializer, the backup).
     *
     */
    class ValueFunction : virtual public ValueFunctionInterface,
                          public BinaryFunction<std::shared_ptr<State>, number, double>

    {
    public:
        ValueFunction();

        ValueFunction(const std::shared_ptr<SolvableByDP> &world,
                      const std::shared_ptr<Initializer> &intializer = nullptr,
                      const std::shared_ptr<ActionSelectionInterface> &action = nullptr);

        /**
         * @brief Copy constructor
         */
        ValueFunction(const ValueFunction &copy);

        /**
         * @brief Get the value at a given state
         */
        virtual double getValueAt(const std::shared_ptr<State> &state, number t = 0) = 0;

        /**
         * @brief Get the q-value at a given state and action
         *
         * @param state the state
         * @param action the action
         * @param t the time step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Call operator to get value of state.
         */
        double operator()(const std::shared_ptr<State> &state, const number &t = 0);

        /**
         * @brief Evaluate the value of an input state
         *
         * @param state the state to evaluate
         * @param t the time step
         * @return a state / value pair 
         */
        virtual Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t) = 0;

        /**
         * @brief Update the value at a given state
         *
         * This function will use the update operator on the input state to make an update
         * to the value function.
         *
         */
        virtual void updateValueAt(const std::shared_ptr<State> &state, number t = 0);

        /**
         * @brief Get the update operator
         */
        std::shared_ptr<UpdateOperatorInterface> getUpdateOperator() const;

        /**
         * @brief Set the update operator
         * 
         * @param update_operator the update operator
         */
        void setUpdateOperator(std::shared_ptr<UpdateOperatorInterface> update_operator);

        /**
         * @brief Get the Init Function
         *
         * @return the function used as default function
         */
        std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> getInitFunction();

        /**
         * @brief Set a function as a interactive way to get initial values for state that are not already initialized.
         *
         * @param init_function the function that enables to get initial values
         */
        void setInitFunction(const std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> &init_function);

        /**
         * @brief Get the total size of the value function
         */
        size_t getSize() const;

        /**
         * @brief Get the size of the value function at timestep t
         */
        virtual size_t getSize(number t) const = 0;

        /**
         * @brief Get a shared pointer on this value function
         */
        std::shared_ptr<ValueFunction> getptr();

        /**
         * @brief Return the possible indexes of the value function
         */
        virtual std::vector<std::shared_ptr<State>> getSupport(number t = 0) = 0;

        /**
         * @brief Copy the value function and return a reference to the copied object.
         * 
         * @return the address of the value function copied
         */
        virtual std::shared_ptr<ValueFunctionInterface> copy() = 0;

    protected:
        /**
         * @brief Initialization function. If defined, algorithms on value functions will get inital values using this function.
         */
        std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> init_function_ = nullptr;

        /**
         * @brief The operator used to update the value function
         */
        std::shared_ptr<UpdateOperatorInterface> update_operator_;
    };
} // namespace sdm