#include <sdm/utils/value_function/update_operator/qupdate/tabular_qupdate.hpp>

namespace sdm
{
    namespace update
    {
        TabularQUpdate::TabularQUpdate(
            std::shared_ptr<ExperienceMemory> experience_memory,
            std::shared_ptr<QValueFunction> q_value_table,
            std::shared_ptr<QValueFunction> target_q_value_table,
            double discount) : QUpdateOperator(q_value_table), experience_memory_(experience_memory), target_q_value_table(target_q_value_table), discount_(discount)
        {
        }

        double TabularQUpdate::update(number t)
        {
            auto [observation, action, reward, next_observation, next_action] = this->experience_memory_->sample(t)[0];

            double delta = reward + this->getWorld()->getDiscount(t) * this->value_function->getQValueAt(next_observation->toState(), next_action, t + 1) - this->q_value_table->getQValueAt(observation->toState(), action, t);

            double new_value = this->value_function->getQValueAt(observation->toState(), action t) + this->learning_rate_ * delta;

            this->value_function->setValueAt(state, action, new_value);

            return delta;
        }
    }
}