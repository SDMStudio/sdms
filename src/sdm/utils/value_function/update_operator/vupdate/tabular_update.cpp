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
            auto new_value = this->value_function->getGreedyActionAndValue(state, t).second;
            // auto old_value = this->value_function->getValueAt(state, t);
            // if (new_value > old_value)
            // {
            //     std::cout << "-- WRONG VALUE -- " << std::endl;
            //     std::cout << "old_value=" << old_value << std::endl;
            //     std::cout << "new_value=" << new_value << std::endl;
            // }
            value_function->setValueAt(state, new_value, t);
        }
    }
}