#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>

namespace sdm
{
    namespace update
    {
        TabularUpdate::TabularUpdate(const std::shared_ptr<ValueFunctionInterface> &value_function)
            : TabularUpdateOperator(value_function)
        {
        }

        void TabularUpdate::update(const std::shared_ptr<State> &state, number t)
        {
            auto new_value = this->getValueFunction()->getGreedyActionAndValue(state, t).second;
            this->getValueFunction()->setValueAt(state, new_value, t);
        }

        void TabularUpdate::update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
        {
            auto new_value = this->getValueFunction()->getQValueAt(state, action, t);
            this->getValueFunction()->setValueAt(state, new_value, t);
        }
    }
}