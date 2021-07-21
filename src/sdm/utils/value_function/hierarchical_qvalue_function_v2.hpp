#pragma once

#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class HierarchicalQValueFunctionV2 : public QValueFunction
    {

    public:
        using Container = std::unordered_map<std::shared_ptr<OccupancyStateInterface>, TabularQValueFunction>;

        HierarchicalQValueFunctionV2(number horizon, double learning_rate, std::shared_ptr<QInitializer> initializer, double ball_r);

        HierarchicalQValueFunctionV2(number horizon = 0, double learning_rate = 0.1, double default_value = 0., double ball_r = 1.0);

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
        double getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t);

        double getQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t, double delta);

        void updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t, double delta);

        bool isNotSeen(const std::shared_ptr<State> &state, number t);

        int getNumStates() const;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, HierarchicalQValueFunctionV2 &vf)
        {
            os << vf.str();
            return os;
        }

    protected:

        number num_agents_ = 2;

        int num_states_ = 0;

        // std::vector<int> num_states_vector_;

        double learning_rate_;

        double ball_r_;

        number horizon_;

        std::vector<std::unordered_map<std::shared_ptr<OccupancyStateInterface>, std::shared_ptr<OccupancyStateInterface>>> closest_s_map_;

        /**
         * @brief The value function represention.
         * 
         */
        std::vector<Container> representation;

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer> initializer_;

        void initializeIfNeeded(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_, number t);

        void initializeQValueFunctionAtWith(const std::shared_ptr<OccupancyStateInterface> &s, number t, const std::shared_ptr<OccupancyStateInterface> &s_);

        void initializeToZeroQValueFunctionAt(const std::shared_ptr<OccupancyStateInterface> &s, number t);

        std::shared_ptr<OccupancyStateInterface> getClosestS(const std::shared_ptr<OccupancyStateInterface> &s, number t);

        bool areInTheSameBall(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_, number t);

    };

} // namespace sdm