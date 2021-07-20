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
#include <sdm/exception.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

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
     * @tparam std::shared_ptr<Observation> Type of the state.
     * @tparam std::shared<Action> Type of the action.
     * @tparam double Type of the value.
     */
    class BaseValueFunction
        : public std::enable_shared_from_this<BaseValueFunction>
    {
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
        virtual void initialize(double v, number t = 0) = 0;

        /**
         * @brief Get the value at a given state
         */
        // virtual double getValueAt(const std::shared_ptr<State> &state, number t = 0) = 0;

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        // virtual std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValueAt(const std::shared_ptr<Observation> &state, number t) = 0;

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        // virtual double getQValueAt(const std::shared_ptr<Observation> &state, const std::shared_ptr<Action> &action, number t) = 0;

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        // virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<Observation> &state, number t) = 0;

        /**
         * @brief Save a value function into a file. 
         * The extension of the file will indicate the type of formatage for recording (`.txt` = text format, '.xml' = XML format, other = binary format). 
         * 
         * @param filename the filename
         */
        virtual void save(std::string) { throw exception::Exception("This class cannot be saved."); }

        /**
         * @brief Load a value function from a file.
         * The extension of the file will indicate the type of formatage for reading (`.txt` = text format, '.xml' = XML format, other = binary format). 
         * 
         * @param filename the filename
         */
        virtual void load(std::string) { throw exception::Exception("This class cannot be load."); }

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const = 0;

        /**
         * @brief Get a shared pointer on the current object
         * 
         * @return the corresponding shared pointer
         */
        std::shared_ptr<BaseValueFunction> getptr();

        number getHorizon() const;

        bool isFiniteHorizon() const;

        bool isInfiniteHorizon() const;

        friend std::ostream &operator<<(std::ostream &os, const BaseValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief The horizon for planning/learning.
         */
        number horizon_;
    };
} // namespace sdm