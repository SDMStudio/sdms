#include <sdm/utils/value_function/update_operator/qupdate/tabular_qupdate.hpp>

namespace sdm
{
    namespace update
    {
        TabularQUpdate::TabularQUpdate(
            std::shared_ptr<ExperienceMemory> experience_memory,
            std::shared_ptr<TabularQValueFunctionInterface> q_value_table,
            std::shared_ptr<TabularQValueFunctionInterface> target_q_value_table,
            double discount,
            double learning_rate) : TabularQUpdateOperator(q_value_table),
                               experience_memory_(experience_memory),
                               target_q_value_table(target_q_value_table),
                               discount_(discount),
                               learning_rate_(learning_rate)
        {
        }

        double TabularQUpdate::deltaSARSA(const std::shared_ptr<State> &observation,
                                          const std::shared_ptr<Action> &action,
                                          double reward,
                                          const std::shared_ptr<State> &next_observation,
                                          const std::shared_ptr<Action> &next_action,
                                          number t)
        {
            return (reward + this->discount_ /* getWorld()->getDiscount(t) */ * qvalue_function->getQValueAt(next_observation, next_action, t + 1) - qvalue_function->getQValueAt(observation, action, t));
        }

        void TabularQUpdate::update(number t)
        {
            auto [observation, action, reward, next_observation, next_action] = this->experience_memory_->sample(t)[0];

            double delta = this->deltaSARSA(observation->toState(), action, reward, next_observation->toState(), next_action, t);

            double new_value = this->qvalue_function->getQValueAt(observation->toState(), action, t) + this->learning_rate_ * delta;

            this->qvalue_function->setQValueAt(observation->toState(), action, new_value, t);
        }
    }
}