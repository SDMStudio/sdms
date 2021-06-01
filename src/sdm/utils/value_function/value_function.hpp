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
#include <boost/serialization/shared_ptr.hpp>

#include <sdm/core/function.hpp>
#include <sdm/utils/value_function/base_value_function.hpp>
#include <sdm/utils/linear_algebra/vector_impl.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

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
     * @tparam std::shared_ptr<State> Type of the state.
     * @tparam std::shared_ptr<Action> Type of the action.
     * @tparam double Type of the value.
     */
    class ValueFunction
        : public BaseValueFunction,
          public BinaryFunction<std::shared_ptr<State>, number, double>

    {
    public:
        ValueFunction() {}

        /**
         * @brief Construct a new Incremental Value Function object
         * 
         * @param problem 
         * @param default_value 
         */
        ValueFunction(std::shared_ptr<SolvableByHSVI>, number);

        /**
         * @brief Destroy the value function
         * 
         */
        virtual ~ValueFunction() {}

        std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> getInitFunction();

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
        void initialize(std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>>);

        /**
         * @brief Get the value at a given state
         */
        virtual double getValueAt(const std::shared_ptr<State> &, number = 0) = 0;

        /**
         * @brief Update the value at a given state
         */
        virtual void updateValueAt(const std::shared_ptr<State> &, number = 0) = 0;

        /**
         * @brief Return the possible indexes of the value function
         * 
         * @return std::string 
         */
        virtual std::vector<std::shared_ptr<State>> getSupport(number) = 0;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() = 0;

        /**
         * @brief Get shared pointer on the current QValueFunction
         */
        std::shared_ptr<ValueFunction> getptr();

        double operator()(const std::shared_ptr<State> &, const number & = 0);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorImpl<std::shared_ptr<Action>, double>> getQValueAt(const std::shared_ptr<State> &, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number);

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        std::shared_ptr<Action> getBestAction(const std::shared_ptr<State> &, number = 0);

        /**
         * @brief Get the world (i.e. the problem that is solve by HSVI).
         * 
         * @return the world
         */
        std::shared_ptr<SolvableByHSVI> getWorld();

        friend std::ostream &operator<<(std::ostream &os, ValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

        /**
         * @brief Get the discount factor. If the problem is serialized then the discount factor is equal to one for every timestep except the one where agent $n$ take an action.  
         * 
         * @param t the timestep
         * @return double the discount factor
         */
        double getDiscount(number);

    protected:
        /**
         * @brief The problem which incremental value function is evaluated 
         * 
         */
        std::shared_ptr<SolvableByHSVI> problem_;

        /**
         * @brief Initialization function. If defined, algorithms on value functions will get inital values using this function.
         * 
         */
        std::shared_ptr<BinaryFunction<std::shared_ptr<State>, number, double>> init_function_ = nullptr;
    };
} // namespace sdm
#include <sdm/utils/value_function/value_function.tpp>