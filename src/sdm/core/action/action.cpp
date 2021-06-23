#include <sdm/core/action/action.hpp>
// #include <sdm/core/action/decision_rule.hpp>
// #include <sdm/core/action/stochastic_decision_rule.hpp>
// #include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/core/action/decision_rule.hpp>

namespace sdm
{

    std::shared_ptr<DecisionRule> Action::toDecisionRule()
    {
        return std::static_pointer_cast<DecisionRule>(this->shared_from_this()->toAction());
    }

    TypeAction Action::getTypeAction() const
    {
        return TypeAction::ACTION;
    }

} // namespace sdm