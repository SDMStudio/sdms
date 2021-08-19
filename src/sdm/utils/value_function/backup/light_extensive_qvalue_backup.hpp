#pragma once

#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/rl/experience_memory.hpp>

#include <sdm/core/space/multi_discrete_space.hpp>

#include <sdm/utils/value_function/light_extensive_qvalue_function.hpp>

#include <sdm/core/action/joint_det_decision_rule.hpp>



namespace sdm
{

    class LightExtensiveQValueBackup : public QValueBackupInterface
    {
    public:

        LightExtensiveQValueBackup();
        LightExtensiveQValueBackup(
            std::shared_ptr<ExperienceMemory> experience_memory, 
            std::shared_ptr<QValueFunction<HistoryJointHistoryPair>> q_value_table, 
            std::shared_ptr<QValueFunction<HistoryJointHistoryPair>> target_q_value_table, 
            double discount, 
            std::shared_ptr<Space> action_space
        );
        
        ~LightExtensiveQValueBackup();
        
        /**
         * @brief Public interface for update() below
         */
        double update(number t);

         /**
         * @brief Public interface for getGreedyAction() below
         */
        std::shared_ptr<Action> getGreedyAction(const std::shared_ptr<State> &state, number t);

        /**
         * @brief 
         * 
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
         */
        double getValueAt(const std::shared_ptr<State> &state, number t);

    protected:
        std::shared_ptr<ExperienceMemory> experience_memory_;
        std::shared_ptr<LightExtensiveQValueFunction> q_value_table_;
        std::shared_ptr<LightExtensiveQValueFunction> target_q_value_table_;
        double discount_;
        number num_agents_ = 2;
        std::shared_ptr<MultiDiscreteSpace> action_space_;

        /**
         * @brief Update Q-value with last collected experience
         * 
         * @param o1 history of agent 1
         * @param o joint history
         * @param u joint action
         * @param r reward
         * @param next_o1 next history of agent 1
         * @param next_o next joint history
         * @param next_u next joint action
         * @param t time-step
         * @return 
         */
        double update(const std::shared_ptr<HistoryInterface> &o1, const std::shared_ptr<JointHistoryInterface> &o,  const std::shared_ptr<Action>& u, double r, 
                      const std::shared_ptr<HistoryInterface> &next_o1, const std::shared_ptr<JointHistoryInterface> &next_o, const std::shared_ptr<Action>& next_u, number t
        );

        /**
         * @brief Get greedy joint decision rule
         * 
         * @param s occupancy state of agent 1
         * @param o1 history of agent 1
         * @param t time-step
         * @return 
         */
        std::shared_ptr<Action> getGreedyAction(const std::shared_ptr<OccupancyStateInterface>& s, const std::shared_ptr<HistoryInterface>& o1, number t);

         /**
         * @brief Get greedy decision rule of agent 2 for every possible action of agent 1
         * 
         * @param s occupancy state of agent 1
         * @param o1 history of agent 1
         * @param t time-step
         * @return 
         */
        std::unordered_map<std::shared_ptr<Item>, std::shared_ptr<DeterministicDecisionRule>> get_a2s(const std::shared_ptr<OccupancyStateInterface>& s, const std::shared_ptr<HistoryInterface> &o1, number t);

        /**
         * @brief Get greedy decision rule of agent 1
         * 
         * @param s occupancy state of agent 1
         * @param o1 history of agent 1
         * @param a2s greedy decision rules of agent 2 for every possible action of agent 1
         * @param t time-step
         * @return 
         */
        std::shared_ptr<DeterministicDecisionRule> get_a1(const std::shared_ptr<OccupancyStateInterface>& s, const std::shared_ptr<HistoryInterface>& o1, const std::unordered_map<std::shared_ptr<Item>, std::shared_ptr<DeterministicDecisionRule>>& a2s, number t);

        // Auxiliary functions
        std::shared_ptr<Action> toJoint(const std::shared_ptr<Item> &u1, const std::shared_ptr<Item> &u2);
        std::shared_ptr<Action> toJoint(const std::shared_ptr<DeterministicDecisionRule> &a1, const std::shared_ptr<DeterministicDecisionRule> &a2);
        std::unordered_map<std::shared_ptr<Item>, std::shared_ptr<DeterministicDecisionRule>> initialize_a2s();
        std::shared_ptr<MappedVector<std::shared_ptr<Item>, double>> initializeQValues();
        std::shared_ptr<DeterministicDecisionRule> initializeDecisionRule();
        std::shared_ptr<Action> getAction(const std::shared_ptr<Action>& a, const std::shared_ptr<JointHistoryInterface>& o);
    };
    

} // namespace sdm
