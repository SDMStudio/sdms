#pragma once

#include <sdm/utils/value_function/qvalue_function.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

namespace sdm
{
    namespace update
    {
        class TabularQUpdate : public TabularUpdateOperator
        {
        public:
            TabularQUpdate(std::shared_ptr<ExperienceMemory> experience_memory, std::shared_ptr<QValueFunction> q_value_table, std::shared_ptr<QValueFunction> target_q_value_table, double discount);

            void update(number t);

        protected:
            std::shared_ptr<ExperienceMemory> experience_memory_;

            std::shared_ptr<QValueFunction> target_q_value_table;

            double discount_;
        };

    } // namespace update
} // namespace sdm
