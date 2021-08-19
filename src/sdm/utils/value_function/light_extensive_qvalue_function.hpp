#pragma once

#include <sdm/utils/value_function/tabular_qvalue_function.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class LightExtensiveQValueFunction : public QValueFunction<HistoryJointHistoryPair>
    {
        
    public:
        using psi = std::unordered_map<std::shared_ptr<HistoryInterface>, TabularQValueFunction<Joint<std::shared_ptr<HistoryInterface>>>>;

        std::vector<psi> Psi;

        LightExtensiveQValueFunction(number horizon, double learning_rate, std::shared_ptr<QInitializer<Joint<std::shared_ptr<HistoryInterface>>>> initializer);
        LightExtensiveQValueFunction(number horizon = 0, double learning_rate = 0.1, double default_value = 0.);


        void initialize();
        void initialize(double v, number t = 0);


        std::shared_ptr<VectorInterface<std::shared_ptr<Action>, double>> getQValuesAt(const HistoryJointHistoryPair &state, number t);

        /**
         * @brief Get the q-value
         * 
         * @param o1 history of agent 1
         * @param o joint history
         * @param u joint action
         * @param t time-step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<HistoryInterface> &o1, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t);
        /**
         * @brief Helper function for the above function
         * 
         * @param o1 history of agent 1
         * @param o joint history
         * @param u joint action
         * @param t time-step
         * @return the q-value
         */
        double getQValueAt(const std::shared_ptr<HistoryInterface> &o1, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t);

        double getQValueAt(const HistoryJointHistoryPair &state, const std::shared_ptr<Action> &action, number t);

        
        void updateQValueAt(const HistoryJointHistoryPair &state, const std::shared_ptr<Action> &action, number t = 0);
        void updateQValueAt(const HistoryJointHistoryPair &state, const std::shared_ptr<Action> &action, number t, double delta);

        /**
         * @brief Update the q-value
         * 
         * @param o1 history of agent 1
         * @param o joint history
         * @param u joint action
         * @param t time-step
         * @return the q-value
         */
        void updateQValueAt(const std::shared_ptr<HistoryInterface> &o1, const Joint<std::shared_ptr<HistoryInterface>> &o, const std::shared_ptr<Action> &u, number t, double delta);
        /**
         * @brief Helper function for the above function
         * 
         * @param o1 history of agent 1
         * @param o joint history
         * @param u joint action
         * @param t time-step
         * @return the q-value
         */
        void updateQValueAt(const std::shared_ptr<HistoryInterface> &o1, const std::shared_ptr<JointHistoryInterface> &o, const std::shared_ptr<Action> &u, number t, double delta);

        bool isNotSeen(const HistoryJointHistoryPair &state, number t);

        int getNumStates() const;

        /**
         * @brief Define this function in order to be able to display the value function
         */
        virtual std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, LightExtensiveQValueFunction &vf)
        {
            os << vf.str();
            return os;
        }

    protected:

        number num_agents_ = 2;
        
        int num_states_ = 0;

        double learning_rate_;

        number horizon_;

        /**
         * @brief The initializer to use for the TabularQValueFunction in Psi
         * 
         */
        std::shared_ptr<QInitializer<Joint<std::shared_ptr<HistoryInterface>>>> initializer_;

        /**
         * @brief Initialize a new Tabular Q Function associated to o1 if o1 was never seen before at t
         * 
         * @param o1 history of agent 1
         * @param t time-step
         * @return the q-value
         */
        void initializeIfNeeded(const std::shared_ptr<HistoryInterface> &o1, number t);
    };

} // namespace sdm