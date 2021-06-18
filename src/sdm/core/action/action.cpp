#include <sdm/core/action/action.hpp>
// #include <sdm/core/action/decision_rule.hpp>
// #include <sdm/core/action/stochastic_decision_rule.hpp>
// #include <sdm/core/action/joint_det_decision_rule.hpp>
#include <sdm/core/action/decision_rule.hpp>

namespace sdm
{
    // std::shared_ptr<DiscreteAction> Action::toDiscreteAction()
    // {
    //     return std::static_pointer_cast<DiscreteAction>(this->shared_from_this());
    // }
    // std::shared_ptr<JointAction> Action::toJointAction()
    // {
    //     return std::static_pointer_cast<JointAction>(this->shared_from_this());
    // }
    // std::shared_ptr<StochasticDecisionRule> Action::toStochasticDecisionRule()
    // {
    //     return std::static_pointer_cast<StochasticDecisionRule>(this->shared_from_this()->toAction());
    // }
    // std::shared_ptr<JointDeterministicDecisionRule> Action::toJointDeterministicDecisionRule()
    // {
    //     return std::static_pointer_cast<JointDeterministicDecisionRule>(this->shared_from_this()->toAction());
    // }
    // std::shared_ptr<DeterministicDecisionRule> Action::toDeterministicDecisionRule()
    // {
    //     return std::static_pointer_cast<DeterministicDecisionRule>(this->shared_from_this()->toAction());
    // }


    std::shared_ptr<DecisionRule> Action::toDecisionRule()
    {
        return std::static_pointer_cast<DecisionRule>(this->shared_from_this()->toAction());
    }
} // namespace sdm