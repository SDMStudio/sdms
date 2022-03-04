#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>
#include <sdm/utils/value_function/update_operator/qupdate/pwlc_qupdate.hpp>

namespace sdm
{
    namespace update
    {
        class SerialPWLCQUpdate : public PWLCQUpdate
        {
        public:
            SerialPWLCQUpdate(std::shared_ptr<ExperienceMemory> experience_memory,
                        std::shared_ptr<ValueFunctionInterface> q_value_table,
                        std::shared_ptr<ValueFunctionInterface> target_q_value);

            void update(double learning_rate, number t);
        };

    } // namespace update
} // namespace sdm
