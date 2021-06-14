#pragma once

// #include <iostream>
// #include <type_traits>
// #include <sdm/types.hpp>
// #include <sdm/core/function.hpp>

#include <sdm/utils/linear_algebra/mapped_matrix.hpp>
#include <sdm/utils/value_function/initializer.hpp>
// #include <sdm/utils/value_function/backup/backup_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class TabularQValueFunction : public QValueFunction
    {

    public:
        using Container = MappedMatrix<std::shared_ptr<Observation>, std::shared_ptr<Action>, double>;

        TabularQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer);

        TabularQValueFunction(number horizon = 0, double learning_rate = 0.1, double default_value = 0.);

        /**
         * @brief Initialize the value function 
         */
        void initialize();

        /**
         * @brief Initialize the value function with a default value
         */
        void initialize(double v, number t = 0);

        /**
         * @brief Get the q-value at a observation
         * 
         * @param observation the observation
         * @return the action value vector 
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const std::shared_ptr<Observation> &observation, number t);

        /**
         * @brief Get the q-value given observation and action
         * 
         * @param observation the observation
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Update the value at a given observation
         */
        void updateQValueAt(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given observation (given a delta)
         */
        void updateQValueAt(const std::shared_ptr<Observation> &observation, const std::shared_ptr<Action> &action, number t, double delta);

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
         * @brief The value function represention.
         * The default representation is a MappedVector but every class implementing VectorInterface interface can be used.
         */
        std::vector<Container> representation;

        double learning_rate_;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer> initializer_;
    };

    

    // template <typename std::shared_ptr<Observation>, typename std::shared_ptr<Action>, typename double = double>
    // using MappedQValueFunction = TabularQValueFunction<std::shared_ptr<Observation>, std::shared_ptr<Action>, double, MappedMatrix>;

    // template <typename std::shared_ptr<Observation>, typename std::shared_ptr<Action>, typename double = double>
    // using SparseValueFunction = TabularQValueFunction<std::shared_ptr<Observation>, std::shared_ptr<Action>, double, ClassicBellmanBackupOperator, SparseVector>;

    // template <typename std::shared_ptr<Observation>, typename std::shared_ptr<Action>, typename double = double>
    // using DenseValueFunction = TabularQValueFunction<std::shared_ptr<Observation>, std::shared_ptr<Action>, double, ClassicBellmanBackupOperator, DenseVector>;

} // namespace sdm