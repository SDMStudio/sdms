#include <sdm/utils/value_function/update_operator/qupdate/tabular_qupdate.hpp>
#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>

namespace sdm
{
    namespace update
    {
        TabularQUpdate::TabularQUpdate(
            std::shared_ptr<ExperienceMemory> experience_memory,
            std::shared_ptr<ValueFunctionInterface> q_value_table,
            std::shared_ptr<ValueFunctionInterface> target_q_value_table,
            double learning_rate) : TabularQUpdateOperator(experience_memory, q_value_table, target_q_value_table, learning_rate)
        {
        }

        double TabularQUpdate::deltaSARSA(const std::shared_ptr<State> &observation,
                                          const std::shared_ptr<Action> &action,
                                          double reward,
                                          const std::shared_ptr<State> &next_observation,
                                          const std::shared_ptr<Action> &next_action,
                                          number t)
        {
            return (reward + getWorld()->getDiscount(t) * this->getQValueFunction()->getQValueAt(next_observation, next_action, t + 1) - this->getQValueFunction()->getQValueAt(observation, action, t));
        }

        void TabularQUpdate::update(number t)
        {
            auto [observation, action, reward, next_observation, next_action] = experience_memory->sample(t)[0];

            double delta = deltaSARSA(observation, action, reward, next_observation, next_action, t);

            double new_value = this->getQValueFunction()->getQValueAt(observation, action, t) + learning_rate * delta;

            this->getQValueFunction()->setQValueAt(observation, action, new_value, t);
        }
    }
}