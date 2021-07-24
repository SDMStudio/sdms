#pragma once

#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/rl/experience_memory.hpp>

#include <sdm/core/space/multi_discrete_space.hpp>

#include <sdm/utils/value_function/hierarchical_qvalue_function.hpp>

#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>


namespace sdm
{

    class SimpleHierarchicalQValueBackup : public QValueBackupInterface
    {
    public:

        SimpleHierarchicalQValueBackup();
        SimpleHierarchicalQValueBackup(
            std::shared_ptr<ExperienceMemoryInterface> experience_memory, 
            std::shared_ptr<QValueFunction> q_value_table, 
            std::shared_ptr<QValueFunction> target_q_value_table, 
            double discount, 
            number horizon,
            std::shared_ptr<Space> action_space,
            std::shared_ptr<GymInterface> env
        );
        
        ~SimpleHierarchicalQValueBackup();
        
        double update();

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
        std::shared_ptr<HierarchicalQValueFunction> q_value_table_;
        std::shared_ptr<HierarchicalQValueFunction> target_q_value_table_;
        std::shared_ptr<PrivateHierarchicalOccupancyMDP> env_;
        double discount_;
        number horizon_;
        number num_agents_ = 2;
        std::shared_ptr<MultiDiscreteSpace> action_space_;
        // All possible joint actions of subordinate agents from the p.o.v. of each agent.
        std::shared_ptr<std::unordered_map<int, std::vector<std::shared_ptr<Joint<std::shared_ptr<Action>>>>>> all_subordinate_jactions;
        void prepareSubordinateJointActions();
        std::shared_ptr<State> getJointHierarchicalHistory(const std::shared_ptr<JointHistoryInterface> &joint_labels, const std::shared_ptr<State> &ostate, number t) const;
        //
        
    };
    

} // namespace sdm
