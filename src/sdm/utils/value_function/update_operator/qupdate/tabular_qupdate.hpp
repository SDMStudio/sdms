#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/utils/value_function/qfunction/tabular_q_interface.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

namespace sdm
{
    namespace update
    {
        class TabularQUpdate : public TabularQUpdateOperator
        {
        public:
            TabularQUpdate(std::shared_ptr<ExperienceMemory> experience_memory,
                           std::shared_ptr<TabularQValueFunctionInterface> q_value_table,
                           std::shared_ptr<TabularQValueFunctionInterface> target_q_value_table,
                           double discount, double learning_rate);

            double deltaSARSA(const std::shared_ptr<State> &observation,
                                              const std::shared_ptr<Action> &action,
                                              double reward,
                                              const std::shared_ptr<State> &next_observation,
                                              const std::shared_ptr<Action> &next_action,
                                              number t);

            void update(number t);

        protected:
            std::shared_ptr<ExperienceMemory> experience_memory_;

            std::shared_ptr<TabularQValueFunctionInterface> target_q_value_table;

            double discount_, learning_rate_;
        };

    } // namespace update
} // namespace sdm
