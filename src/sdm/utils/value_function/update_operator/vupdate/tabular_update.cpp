#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/vfunction/tabular_value_function.hpp>
#include <sdm/utils/value_function/update_operator/vupdate/tabular_update.hpp>

namespace sdm
{
namespace update
{
    TabularUpdate::TabularUpdate(const std::shared_ptr<SolvableByHSVI> &world, const std::shared_ptr<TabularValueFunctionInterface> &value_function)
        : TabularUpdateOperator(value_function)
    {
    }

    void TabularUpdate::update(std::shared_ptr<State> state, std::shared_ptr<Action> action, number t)
    {
        value_function->setValueAt(state, this->value_function->getQValueAt(state, action, t), t);
    }
}
}