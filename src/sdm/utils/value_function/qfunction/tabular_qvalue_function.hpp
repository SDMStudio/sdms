#pragma once

#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief Q-value function instance represented by a mapping function.
     * 
     * This representation can be used with all types of states and actions. A value is assign
     * to each pair (state, action) where the state must inherit from the `State` interface and
     * the action must inherit from the `Action` interface.
     * 
     */
    class TabularQValueFunction : public QValueFunction, public TabularQValueFunctionInterface
    {
    public:
        using Container = MappedMatrix<std::shared_ptr<State>, std::shared_ptr<Action>, double>;

        TabularQValueFunction(const std::shared_ptr<SolvableByDP> &world,
                              const std::shared_ptr<Initializer> &initializer,
                              const std::shared_ptr<ActionSelectionInterface> &action_selection,
                              const std::shared_ptr<TabularQUpdateOperator> &update_operator = nullptr);

        /**
         * @brief Initialize the value function by using initializer.
         */
        void initialize();

        /**
         * @brief Set all values of the vector to a default value.
         *
         * @param default_value the default value
         */
        void initialize(double default_value, number t = 0);

        /**
         * @brief Get the q-value at a specific state, action and time step.
         * 
         * @param state the state
         * @param action the action
         * @param t the time step
         * @return the q-value 
         */
        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Set the q-value at a specific state s, action a and timestep t.
         *
         * @param state the state
         * @param action the action
         * @param t the time step
         */
        void setQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double value, number t);

        /**
         * @brief Get the q-value at a state
         * 
         * @param state the state
         * @return the action value vector 
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const std::shared_ptr<State> &state, number t);

        int getNumStates() const;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, TabularQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:
        /**
         * @brief The Q value function represention.
         * 
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         *
         */
        std::vector<Container> representation;
    };
} // namespace sdm
