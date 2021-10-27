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
#include <sdm/public/boost_serializable.hpp>
#include <sdm/world/solvable_by_dp.hpp>
#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/action_selection/action_selection_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief This class is the abstract class of all kind of value functions. 
     * 
     * All {state,action,q}-value function must inherit from this class.
     * 
     * Value functions can :
     * 
     * - be initialized --> thanks to the initializer 
     * - be updated --> thanks to the update operator 
     * - be used to select greedy action --> thanks to the action selection operator 
     * 
     */
    class ValueFunctionInterface : public std::enable_shared_from_this<ValueFunctionInterface>
    {
    public:
        /**
         * @brief Default constructor
         * 
         */
        ValueFunctionInterface();

        /**
         * @brief Construct a new value function with specific initializer, action selector and update operator.
         * 
         * @param horizon the horizon
         * @param initializer the initializer
         * @param action_selection the action selection operator
         * @param update_operator the update operator
         */
        ValueFunctionInterface(const std::shared_ptr<SolvableByDP> &world,
                               const std::shared_ptr<Initializer> &initializer,
                               const std::shared_ptr<ActionSelectionInterface> &action_selection);

        /**
         * @brief Destroy the value function
         */
        virtual ~ValueFunctionInterface();

        /**
         * @brief Initialize the value function 
         */
        virtual void initialize();

        /**
         * @brief Initialize the value function to a constant value
         */
        virtual void initialize(double v, number t = 0) = 0;

        /**
         * @brief Get the value at a given state
         */
        virtual double getValueAt(const std::shared_ptr<State> &state, number t = 0) = 0;

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        virtual double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) = 0;

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @param t the time step
         * @return the best action
         */
        virtual std::shared_ptr<Action> getGreedyAction(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Select the best Action and the value associated for a precise state and a time step
         * 
         * @param state the state
         * @param t the time step
         * @return the best action and the corresponding value
         */
        virtual Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Get the corresponding world
         * 
         * @return the world 
         */
        virtual std::shared_ptr<SolvableByDP> getWorld() const;

        /**
         * @brief Save a value function into a file. 
         * 
         * The extension of the file will indicate the type of formatage for 
         * recording (`.txt` = text format, '.xml' = XML format, other = binary 
         * format). 
         * 
         * @param filename the filename
         * 
         */
        virtual void save(std::string);

        /**
         * @brief Load a value function from a file.
         * 
         * The extension of the file will indicate the type of formatage for 
         * reading (`.txt` = text format, '.xml' = XML format, other = binary 
         * format). 
         * 
         * @param filename the filename
         * 
         */
        virtual void load(std::string);

        /**
         * @brief Define this function in order to be able to display the value 
         * function
         */
        virtual std::string str() const = 0;

        /**
         * @brief Get a shared pointer on this value function
         */
        std::shared_ptr<ValueFunctionInterface> getptr();

        /**
         * @brief Get the horizon
         */
        number getHorizon() const;

        /**
         * @brief Checks if the problem has a finite horizon.
         * 
         * The problem has a finite horizon if the horizon is set to a value 
         * greater than zero.
         * 
         */
        bool isFiniteHorizon() const;

        /**
         * @brief Checks if the problem has an infinite horizon.
         * 
         * The problem has an infinite horizon if the horizon is set to zero.
         * 
         */
        bool isInfiniteHorizon() const;

        /**
         * @brief Get the initializer 
         */
        std::shared_ptr<Initializer> getInitializer() const;

        /**
         * @brief Get the action selection operator
         */
        std::shared_ptr<ActionSelectionInterface> getActionSelection() const;

        friend std::ostream &operator<<(std::ostream &os, const ValueFunctionInterface &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief The horizon for planning/learning.
         */
        number horizon_;

        /**
         * @brief The world 
         * 
         */
        std::shared_ptr<SolvableByDP> world_;

        /**
         * @brief The initializer to use for this value function. 
         */
        std::shared_ptr<Initializer> initializer_;

        /**
         * @brief The operator used to select the action.
         */
        std::shared_ptr<ActionSelectionInterface> action_selection_;
    };

} // namespace sdm
