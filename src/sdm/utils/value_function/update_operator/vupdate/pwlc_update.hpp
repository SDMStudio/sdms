
#pragma once

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

        protected:
            std::shared_ptr<State> computeNewHyperplane(const std::shared_ptr<BeliefInterface> &belief_state, number t);
            std::shared_ptr<State> computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);
        };
    }
}