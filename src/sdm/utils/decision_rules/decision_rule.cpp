#include <sdm/utils/decision_rules/decision_rule.hpp>

namespace sdm
{
    template <typename state_t, typename action_t>
    action_t DecisionRule<state_t, action_t>::act(const state_t &state) const
    {
        this->model(state);
    }

} // namespace sdm
