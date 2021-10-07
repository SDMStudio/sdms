#pragma once

#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/rl/experience_memory.hpp>

namespace sdm
{

    class TabularQValueBackup : public QValueBackupInterface
    {
    public:

        TabularQValueBackup();
        TabularQValueBackup(std::shared_ptr<ExperienceMemory> experience_memory, std::shared_ptr<QValueFunction> q_value_table, std::shared_ptr<QValueFunction> target_q_value_table, double discount);
        
        virtual ~TabularQValueBackup();
        
        /**
         * @brief 
         * 
         * @param number t : time step
         * @return 
         */
        double update(number t);

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

        std::shared_ptr<QValueFunction> q_value_table, target_q_value_table;
        
        double discount_;
    };
    

} // namespace sdm
