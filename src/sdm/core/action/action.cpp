#include <sdm/core/action/action.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{

    size_t Action::hash(double precision) const
    {
        throw exception::Exception("Hash (i.e. size_t X::hash() const ) is not implemented for this class");
    }

    bool Action::isEqual(const std::shared_ptr<Action> &, double) const
    {
        throw exception::Exception("Equal Operator (i.e. bool X::isEqual() const ) is not implemented for this class");
    }

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

} // namespace sdm