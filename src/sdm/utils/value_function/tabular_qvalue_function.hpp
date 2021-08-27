#pragma once

#include <sdm/utils/linear_algebra/mapped_matrix.hpp>

#include <sdm/utils/value_function/initializer/initializer.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    template <class TInput = std::shared_ptr<State>>
    class TabularQValueFunction : public QValueFunction<TInput>
    {

    public:
        using Container = MappedMatrix<TInput, std::shared_ptr<Action>, double>;

        TabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<TInput>> initializer, bool active_learning = true);

        TabularQValueFunction(number horizon = 0, double learning_rate = 0.1, double default_value = 0., bool active_learning = true);

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
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const TInput &state, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t);

        // double getValueAt(const TInput &state, number t);

        // std::shared_ptr<Action> getBestAction(const TInput &state, number t = 0);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const TInput &state, const std::shared_ptr<Action> &action, number t, double delta);

        bool isNotSeen(const TInput &state, number t);

        int getNumStates() const;

        // void printNumberOfActions();
        // size_t getSize() const
        // {
        //     size_t size_tot = 0;
        //     for (const auto &repr : this->representation)
        //     {
        //         for (const auto &pair_action_value_funct : repr)
        //         {
        //             size_tot += pair_action_value_funct.second.size();
        //         }
        //     }
        //     return size_tot;
        // }

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
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<Container> representation;

        double learning_rate_;
        bool active_learning_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer<TInput>> initializer_;
    };
} // namespace sdm
#include <sdm/utils/value_function/tabular_qvalue_function.tpp>
