#pragma once

#include <sdm/utils/value_function/backup/qvalue_backup_interface.hpp>
#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/utils/rl/experience_memory.hpp>

namespace sdm
{

    class TabularQValueBackup : public QValueBackupInterface
    {
    public:

        TabularQValueBackup();
        TabularQValueBackup(std::shared_ptr<ExperienceMemory> experience_memory, std::shared_ptr<QValueFunction> q_value_table, std::shared_ptr<QValueFunction> target_q_value_table, double discount);
        
        // ~TabularQValueBackup();
        
        /**
         * @brief 
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return 
         */
        double backup(number t);
    protected:
        std::shared_ptr<ExperienceMemory> experience_memory_;
        std::shared_ptr<QValueFunction> q_value_table_;
        std::shared_ptr<QValueFunction> target_q_value_table_;
        double discount_;
    };
    

} // namespace sdm
