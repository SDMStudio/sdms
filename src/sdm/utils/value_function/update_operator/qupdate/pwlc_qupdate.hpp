#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/value_function/update_operator/qupdate_operator.hpp>

namespace sdm
{
    namespace update
    {
        class PWLCQUpdate : public PWLCQUpdateOperator
        {
        public:
            PWLCQUpdate(std::shared_ptr<ExperienceMemory> experience_memory,
                        std::shared_ptr<ValueFunctionInterface> q_value_table,
                        std::shared_ptr<ValueFunctionInterface> target_q_value,
                        double learning_rate);

            double target(const std::shared_ptr<State> &observation,
                          const std::shared_ptr<Action> &action,
                          double reward,
                          const std::shared_ptr<State> &next_observation,
                          const std::shared_ptr<Action> &next_action,
                          number t);

            double delta(const std::shared_ptr<State> &observation,
                         const std::shared_ptr<Action> &action,
                         double reward,
                         const std::shared_ptr<State> &next_observation,
                         const std::shared_ptr<Action> &next_action,
                         number t);

            void update(number t);

            std::shared_ptr<State> computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<DecisionRule> &decision_rule, double delta, number t);
        };

    } // namespace update
} // namespace sdm