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
        typename ExperienceMemory::sars_transition transition = this->experience_memory_->sample(t)[0];

        std::shared_ptr<Observation> observation = std::get<0>(transition);
        std::shared_ptr<Action> action = std::get<1>(transition);
        double reward = std::get<2>(transition);
        std::shared_ptr<Observation> next_observation = std::get<3>(transition);

        double q_value = this->q_value_table_->getQValueAt(observation, action, t);
        double next_value = this->target_q_value_table_->getNextValueAt(next_observation, t + 1);
        double target_q_value = reward + this->discount_ * next_value;
        double delta = target_q_value - q_value;
        this->q_value_table_->updateQValueAt(observation, action, t, delta);
        return delta;
    }

}