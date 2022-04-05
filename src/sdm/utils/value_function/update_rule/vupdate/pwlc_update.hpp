
#pragma once

#include <sdm/utils/linear_algebra/hyperplane/hyperplane.hpp>
#include <sdm/utils/value_function/update_rule/vupdate_rule.hpp>

//TODO importer FactoredOccupancyState

namespace sdm
{
    namespace update
    {
        class PWLCUpdate : public PWLCUpdateRule
        {
        public:
            PWLCUpdate(const std::shared_ptr<ValueFunctionInterface> &value_function);

            void update(const std::shared_ptr<State> &state, number t);
            void update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
            void update(const std::shared_ptr<State> &state, double new_value, number t){}

        protected:
            //TODO penser a l'option template. Mais pour cela, il faut que toutes les deux fonctions aient les familles d'arguments.
            std::shared_ptr<Hyperplane> computeNewHyperplane(const std::shared_ptr<BeliefInterface> &belief_state, number t);
            std::shared_ptr<Hyperplane> computeNewHyperplane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t);
        };
    }
}