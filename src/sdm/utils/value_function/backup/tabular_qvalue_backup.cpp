#include <sdm/utils/value_function/backup/tabular_qvalue_backup.hpp>

namespace sdm
{
    // TabularQValueBackup::TabularQValueBackup()
    // {

    // }

    TabularQValueBackup::TabularQValueBackup(
        std::shared_ptr<ExperienceMemory> experience_memory, 
        std::shared_ptr<QValueFunction> q_value_table, 
        std::shared_ptr<QValueFunction> target_q_value_table, 
        double discount
    ) : experience_memory_(experience_memory), q_value_table_(q_value_table), target_q_value_table_(target_q_value_table), discount_(discount)
    {

    }

    // TabularQValueBackup::~TabularQValueBackup(){}

    double TabularQValueBackup::backup(number t)
    {   
        std::tie(this->observation, this->action, this->reward, this->next_observation) = this->experience_memory_->sample(t)[0];
        double q_value = this->q_value_table_->getQValueAt(this->observation, this->action, t);
        double next_value = this->target_q_value_table_->getValueAt(this->next_observation, t + 1);
        double target_q_value = reward + this->discount_ * next_value;
        double delta = target_q_value - q_value;
        this->q_value_table_->updateQValueAt(this->observation, this->action, t, delta);
        return delta;
    }

}