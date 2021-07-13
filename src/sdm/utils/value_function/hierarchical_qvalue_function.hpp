#pragma once

#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
#include <sdm/core/state/interface/joint_history_interface.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class HierarchicalQValueFunction : public QValueFunction
    {

    public:
        using Container = TabularQValueFunction;

        HierarchicalQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer);

        HierarchicalQValueFunction(number horizon = 0, double learning_rate = 0.1, double default_value = 0.);

        /**
         * @brief Initialize the value function 
         */
        void initialize();

        /**
         * @brief Initialize the value function with a default value
         */
        void initialize(double v, number t = 0);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        double getValueAt(const std::shared_ptr<State> &state, number t);

        std::shared_ptr<Action> getBestAction(const std::shared_ptr<State> &state, number t = 0);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta);

        bool isNotSeen(const std::shared_ptr<State> &state, number t);

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, HierarchicalQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:

        /**
         * @brief The value function represention.
         * 
         */
        std::unordered_map<std::shared_ptr<State>, Container> representation;

        double learning_rate_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer> initializer_;
    };

} // namespace sdm