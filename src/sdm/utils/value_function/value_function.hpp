#pragma once

#include <sdm/core/function.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

#include <sdm/utils/value_function/base_value_function.hpp>
#include <sdm/utils/value_function/backup_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    // class SolvableByHSVI;
    
    /**
     * @class ValueFunction
     * @brief This class is the abstract class of value function. All value function must derived this class.
     * 
     * @tparam std::shared_ptr<Item> Type of the state.
     * @tparam std::shared_ptr<Action> Type of the action.
     * @tparam double Type of the value.
     */
    class ValueFunction
        : public BaseValueFunction,
          public BinaryFunction<std::shared_ptr<Item>, number, double>

    {
    public:
        ValueFunction() {}

        /**
         * @brief Construct a new Incremental Value Function object
         * 
         * @param backup 
         * @param default_value 
         */
        ValueFunction(std::shared_ptr<BackupInterface>, number);

        /**
         * @brief Destroy the value function
         * 
         */
        virtual ~ValueFunction() {}

        std::shared_ptr<BinaryFunction<std::shared_ptr<Item>, number, double>> getInitFunction();

        /**
         * @brief Initialize the value function 
         */
        virtual void initialize() = 0;

        /**
         * @brief Initialize the value function with a default value
         */
        virtual void initialize(double v, number t = 0) = 0;

        /**
         * @brief Set a function as a interactive way to get initial values for state that are not already initialized. 
         * 
         * @param init_function the function that enables to get initial values 
         */
        void initialize(std::shared_ptr<BinaryFunction<std::shared_ptr<Item>, number, double>>);

        /**
         * @brief Get the value at a given state
         */
        virtual double getValueAt(const std::shared_ptr<Item> &, number = 0) = 0;

        /**
         * @brief Update the value at a given state
         */
        virtual void updateValueAt(const std::shared_ptr<Item> &, number = 0) = 0;

        /**
         * @brief Return the possible indexes of the value function
         * 
         * @return std::string 
         */
        virtual std::vector<std::shared_ptr<Item>> getSupport(number) = 0;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const = 0;

        /**
         * @brief Get shared pointer on the current QValueFunction
         */
        std::shared_ptr<ValueFunction> getptr();

        double operator()(const std::shared_ptr<Item> &, const number & = 0);

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        std::shared_ptr<Action> getBestAction(const std::shared_ptr<Item> &, number = 0);

        /**
         * @brief Get the world (i.e. the problem that is solve by HSVI).
         * 
         * @return the world
         */
        // std::shared_ptr<SolvableByHSVI> getWorld();

    protected:

        /**
         * @brief Initialization function. If defined, algorithms on value functions will get inital values using this function.
         * 
         */
        std::shared_ptr<BinaryFunction<std::shared_ptr<Item>, number, double>> init_function_ = nullptr;

        std::shared_ptr<BackupInterface> backup_;
    };
} // namespace sdm