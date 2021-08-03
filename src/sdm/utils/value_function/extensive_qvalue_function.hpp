#pragma once

#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class ExtensiveQValueFunction : public QValueFunction<OccupancyStateJointHistoryPair>
    {

    public:
        using psi = std::unordered_map<std::shared_ptr<OccupancyStateInterface>, TabularQValueFunction<Joint<std::shared_ptr<HistoryInterface>>>>;

        std::vector<psi> Psi;

        ExtensiveQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<Joint<std::shared_ptr<HistoryInterface>>>> initializer, double ball_r, bool keep_map);

        ExtensiveQValueFunction(number horizon = 0, double learning_rate = 0.1, double default_value = 0., double ball_r = 1.0, bool keep_map = true);

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
        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const OccupancyStateJointHistoryPair &state, number t);

        /**
         * @brief Get the q-value given state and action
         * 
         * @param state the state
         * @param action the action
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t);

        double getQValueAt(const OccupancyStateJointHistoryPair &state, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Update the value at a given state
         */
        void updateQValueAt(const OccupancyStateJointHistoryPair &state, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Update the value at a given state (given a delta)
         */
        void updateQValueAt(const OccupancyStateJointHistoryPair &state, const std::shared_ptr<Action> &action, number t, double delta);

        void updateQValueAt(const std::shared_ptr<OccupancyStateInterface> &s, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t, double delta);

        bool isNotSeen(const OccupancyStateJointHistoryPair &state, number t);

        int getNumStates() const;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, ExtensiveQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:

        number num_agents_ = 2;

        int num_key_states_ = 0;
        int num_states_ = 0;

        // std::vector<int> num_states_vector_;

        double learning_rate_;

        double ball_r_;

        number horizon_;

        bool keep_map_;

        std::vector<std::unordered_map<std::shared_ptr<OccupancyStateInterface>, std::shared_ptr<OccupancyStateInterface>>> closest_s_map_;

        /**
         * @brief The value function represention.
         * 
         */

        /**
         * @brief The initializer to use for this value function. 
         * 
         */
        std::shared_ptr<QInitializer<Joint<std::shared_ptr<HistoryInterface>>>> initializer_;

        void initializeQValueFunctionAtWith(const std::shared_ptr<OccupancyStateInterface> &s, TabularQValueFunction<Joint<std::shared_ptr<HistoryInterface>>> &q_, number t);

        void initializeToZeroQValueFunctionAt(const std::shared_ptr<OccupancyStateInterface> &s, number t);

        std::shared_ptr<OccupancyStateInterface> getHyperPlaneIndex(const std::shared_ptr<OccupancyStateInterface> &s, number t);

        bool areInTheSameBall(const std::shared_ptr<OccupancyStateInterface> &s, const std::shared_ptr<OccupancyStateInterface> &s_, number t);

    };

} // namespace sdm