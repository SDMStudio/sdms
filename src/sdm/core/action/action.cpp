#include <sdm/core/action/action.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{
    // std::shared_ptr<JointAction> Action::toJointAction()
    // {
    //     return std::static_pointer_cast<JointAction>(this->getPointer());
    // }

    std::shared_ptr<DecisionRule> Action::toDecisionRule()
    {
        return std::static_pointer_cast<DecisionRule>(this->getPointer());
    }

    std::shared_ptr<JointDeterministicDecisionRule> Action::toJointDeterministicDecisionRule()
    {
        return std::dynamic_pointer_cast<JointDeterministicDecisionRule>(this->getPointer());
    }

    std::shared_ptr<JointAction> Action::toJointAction()
    {
        return std::static_pointer_cast<JointAction>(this->getPointer());
    }

    TypeAction Action::getTypeAction() const
    {
        return TypeAction::ACTION;
    }

} // namespace sdm