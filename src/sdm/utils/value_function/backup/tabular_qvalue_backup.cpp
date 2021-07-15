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
        // std::cout << "-------- TabularQValueBackup::backup() ---------" << std::endl;
        auto [observation, action, reward, next_observation] = this->experience_memory_->sample(t)[0];
        // std::cout << "-------- TabularQValueBackup::backup() ---------1" << std::endl;
        // std::cout << *observation << std::endl;
        // std::cout << *action << std::endl;
        // std::cout << this->q_value_table_ << std::endl;
        // std::cout << *this->q_value_table_ << std::endl;
        // std::cout << *observation->toState() << std::endl;
        // std::cout << "-------- TabularQValueBackup::backup() ---------1.5" << std::endl;
        

        double q_value = this->q_value_table_->getQValueAt(observation->toState(), action, t);
        // std::cout << "-------- TabularQValueBackup::backup() ---------2" << std::endl;

        double next_value = this->target_q_value_table_->getValueAt(next_observation->toState(), t + 1);
        // std::cout << "-------- TabularQValueBackup::backup() ---------3" << std::endl;

        double target_q_value = reward + this->discount_ * next_value;
        // std::cout << "-------- TabularQValueBackup::backup() ---------4" << std::endl;

        double delta = target_q_value - q_value;
        // std::cout << "-------- TabularQValueBackup::backup() ---------5" << std::endl;

        this->q_value_table_->updateQValueAt(observation->toState(), action, t, delta);
        // std::cout << "-------- TabularQValueBackup::backup() ---------6" << std::endl;

        return delta;
    }

}