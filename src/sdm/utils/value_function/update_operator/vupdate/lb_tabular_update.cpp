#include <sdm/utils/value_function/update_operator/vupdate/lb_tabular_update.hpp>

namespace sdm
{
    namespace update
    {
        LowerBoundTabularUpdate::LowerBoundTabularUpdate(const std::shared_ptr<ValueFunctionInterface> &value_function)
            : TabularUpdate(value_function)
        {
        }

        void LowerBoundTabularUpdate::update(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t)
        {
            auto old_value = this->getValueFunction()->getValueAt(state, t);
            auto new_value = this->getValueFunction()->getQValueAt(state, action, t);
            this->getValueFunction()->setValueAt(state, std::max(new_value, old_value), t);
        }
    }
}