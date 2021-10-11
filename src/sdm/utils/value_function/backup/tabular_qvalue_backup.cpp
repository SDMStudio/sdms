#include <sdm/utils/value_function/backup/tabular_qvalue_backup.hpp>

namespace sdm
{
    TabularQValueBackup::TabularQValueBackup()
    {
    }

    TabularQValueBackup::TabularQValueBackup(
        std::shared_ptr<ExperienceMemory> experience_memory,
        std::shared_ptr<QValueFunction> q_value_table,
        std::shared_ptr<QValueFunction> target_q_value_table,
        double discount) : experience_memory_(experience_memory), q_value_table(q_value_table), target_q_value_table(target_q_value_table), discount_(discount)
    {
    }

    TabularQValueBackup::~TabularQValueBackup()
    {
    }

    double TabularQValueBackup::update(number t)
    {
        auto [observation, action, reward, next_observation, next_action] = this->experience_memory_->sample(t)[0];
        double delta = reward + this->discount_ * this->q_value_table->getQValueAt(next_observation->toState(), next_action, t + 1) - this->q_value_table->getQValueAt(observation->toState(), action, t);
        this->q_value_table->updateQValueAt(observation->toState(), action, delta, t);
        return delta;
    }

    std::shared_ptr<Action> TabularQValueBackup::getGreedyAction(const std::shared_ptr<State> &state, number t)
    {
        std::cout << "ERROR !!! Ne devrait pas passer par la !" << std::endl;
    }

    double TabularQValueBackup::getValueAt(const std::shared_ptr<State> &state, number t)
    {
        return this->target_q_value_table->getQValuesAt(state, t)->max();
    }

}