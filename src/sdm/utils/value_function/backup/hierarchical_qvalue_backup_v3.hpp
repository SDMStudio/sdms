#pragma once

#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/rl/experience_memory.hpp>

#include <sdm/core/space/multi_discrete_space.hpp>

#include <sdm/utils/value_function/hierarchical_qvalue_function_v2.hpp>

namespace sdm
{

    class HierarchicalQValueBackupV3 : public QValueBackupInterface
    {
    public:

        HierarchicalQValueBackupV3();
        HierarchicalQValueBackupV3(std::shared_ptr<ExperienceMemory> experience_memory, std::shared_ptr<QValueFunction> q_value_table, std::shared_ptr<QValueFunction> target_q_value_table, double discount, std::shared_ptr<Space> action_space);
        
        ~HierarchicalQValueBackupV3();
        
        /**
         * @brief 
         * 
         * @param number t : time step
         * @return 
         */
        double backup(number t);

        /**
         * @brief 
         * 
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
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
        std::shared_ptr<HierarchicalQValueFunctionV2> q_value_table_;
        std::shared_ptr<HierarchicalQValueFunctionV2> target_q_value_table_;
        double discount_;
        number num_agents_ = 2;
        std::shared_ptr<MultiDiscreteSpace> action_space_;
        // All possible joint actions of subordinate agents from the p.o.v. of each agent.
        std::shared_ptr<std::unordered_map<int, std::vector<std::shared_ptr<Joint<std::shared_ptr<Action>>>>>> all_subordinate_jactions;
        double getQValueAt(const std::shared_ptr<OccupancyStateInterface> &s_, const std::shared_ptr<OccupancyStateInterface> &s, std::shared_ptr<JointDeterministicDecisionRule> &a, number t);
        std::shared_ptr<JointDeterministicDecisionRule> getPossibleGreedyAction(const std::shared_ptr<OccupancyStateInterface> &s, number t);
        void prepareSubordinateJointActions();
        std::shared_ptr<State> getJointHierarchicalHistory(const std::shared_ptr<JointHistoryInterface> &joint_labels, const std::shared_ptr<State> &ostate, number t) const;
    };
    

} // namespace sdm
