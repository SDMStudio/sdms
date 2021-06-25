#include <sdm/core/action/action.hpp>
#include <sdm/core/action/decision_rule.hpp>

namespace sdm
{

    std::shared_ptr<DecisionRule> Action::toDecisionRule()
    {
        return std::dynamic_pointer_cast<DecisionRule>(this->getPointer());
    }

    TypeAction Action::getTypeAction() const
    {
        return TypeAction::ACTION;
    }

} // namespace sdm