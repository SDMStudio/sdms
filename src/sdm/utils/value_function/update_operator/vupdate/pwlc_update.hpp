
#pragma once

#include <sdm/utils/linear_algebra/hyperplane/hyperplane.hpp>
#include <sdm/utils/value_function/update_operator/vupdate_operator.hpp>

namespace sdm
{
    namespace update
    {
        class PWLCUpdate : public PWLCUpdateOperator
        {
        public:
            PWLCUpdate(const std::shared_ptr<ValueFunctionInterface> &value_function);

            void update(const std::shared_ptr<State> &state, number t);
            void update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
            void update(const std::shared_ptr<State> &state, double new_value, number t){}

        protected:
            std::shared_ptr<Hyperplane> computeNewHyperplane(const std::shared_ptr<BeliefInterface> &belief_state, number t);
            std::shared_ptr<Hyperplane> computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t);
        };
    }
}