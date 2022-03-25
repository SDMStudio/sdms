#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/rl/experience_memory.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

namespace sdm
{
    namespace update
    {
        class TabularQUpdate : public TabularQUpdateOperator
        {
        public:
            TabularQUpdate(std::shared_ptr<ExperienceMemory> experience_memory,
                           std::shared_ptr<ValueFunctionInterface> q_value_table,
                           std::shared_ptr<ValueFunctionInterface> target_q_value_table);

            double deltaSARSA(const std::shared_ptr<State> &observation,
                                              const std::shared_ptr<Action> &action,
                                              double reward,
                                              const std::shared_ptr<State> &next_observation,
                                              const std::shared_ptr<Action> &next_action,
                                              number t);

            void update(double learning_rate, number t);
        };

    } // namespace update
} // namespace sdm
