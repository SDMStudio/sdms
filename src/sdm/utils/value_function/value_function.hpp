#pragma once

#include <sdm/types.hpp>
#include <sdm/core/function.hpp>
#include <sdm/utils/value_function/value_function_interface.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>
#include <sdm/utils/value_function/action_selection/action_selection_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

    using namespace update;
    
    /**
     * @class ValueFunction
     * @brief This class contains attributes and methods common to all value functions. 
     * 
     * Some attributes are callable. They will be called to update the value function (i.e. the initializer, the backup).  
     * 
     */
    class ValueFunction
        : public ValueFunctionInterface,
          public BinaryFunction<std::shared_ptr<State>, number, double>

    {
    public:
        ValueFunction();

        /**
         * @brief Construct an incremental value function object.
         * 
         * @param horizon the horizon
         * @param intializer the initializer function
         * @param backup the backup function
         * @param action the action selection function
         */
        ValueFunction(number horizon = 0, const std::shared_ptr<Initializer> &intializer = nullptr,
                      const std::shared_ptr<ActionSelectionInterface> &action = nullptr,
                      const std::shared_ptr<UpdateOperatorInterface> &update_operator = nullptr);

        /**
         * @brief Copy constructor
         * 
         * @param copy the value function to be copied
         */
        ValueFunction(const ValueFunction &copy);

        /**
         * @brief Destroy the value function
         * 
         */
        virtual ~ValueFunction();

        /**
         * @brief Initialize the value function
         * 
         */
        void initialize();

        /**
         * @brief Initialize the value function with a default value
         */
        virtual void initialize(double v, number t = 0) = 0;

        /**
         * @brief Set a function as a interactive way to get initial values for state that are not already initialized. 
         * 
         * @param init_function the function that enables to get initial values 
         */
        void initialize(const std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> &init_function);

        /**
         * @brief Get the q-value at a given state and action
         * 
         * @param state the state
         * @param action the action
         * @param t the time step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /** @brief Call operator to get value of state. */
        double operator()(const std::shared_ptr<State> &state, const number &t = 0);

        /**
         * @brief Get the Init Function 
         * 
         * @return the function used as default function
         */
        std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> getInitFunction();

        /**
         * @brief Evaluate the element given
         * 
         * @param state : ELement to evaluate
         * @param t 
         * @return Pair<std::shared_ptr<State>, double> 
         */
        virtual Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<State> &state, number t) = 0;

        /**
         * @brief Update the value at a given state
         * 
         * This function will use the update operator on the input state to make an update 
         * to the value function.
         * 
         */
        virtual void updateValueAt(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Update the value at a given state and action.
         */
        virtual void updateValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Get the update operator
         */
        std::shared_ptr<UpdateOperatorInterface> getUpdateOperator() const;

        /**
         * @brief Return the possible indexes of the value function
         */
        virtual std::vector<std::shared_ptr<State>> getSupport(number t) = 0;

        /** @brief Get the size of the value function at timestep t */
        virtual size_t getSize(number t) const = 0;

        /** @brief Get the total size of the value function. */
        size_t getSize() const;

        /**
         * @brief Prune unecessary data at a specific time step.
         * 
         * @param t the time step
         */
        virtual void do_pruning(number t) = 0;

        std::shared_ptr<ValueFunction> getptr();

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