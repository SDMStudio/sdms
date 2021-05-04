
#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{

    /**
   * @brief The joint deterministic decision rule class. This class is a function that maps joint generic states to joint generic actions. 
   * 
   * @tparam TState the state type
   * @tparam TAction the action type
   */
    template <typename TState, typename TAction>
    class PrivateJointDeterministicDecisionRule
        : public JointDeterministicDecisionRule<TState, TAction>
    {
    public:
        using input_type = TState;
        using output = TAction;
        using output_type = typename Function<Joint<TState>, Joint<TAction>>::output_type;

        PrivateJointDeterministicDecisionRule();
        PrivateJointDeterministicDecisionRule(std::vector<std::vector<TState>> acc_states, std::vector<std::vector<TAction>> actions);
        PrivateJointDeterministicDecisionRule(const std::vector<DeterministicDecisionRule<TState, TAction>> &idr_list, number action_agent_i = 0, number agent_id = 0);

        Joint<TAction> act(const Joint<TState> &jobserv) const;

        void setProbability(const std::vector<TState> &, const std::vector<TAction> &);

        friend std::ostream &operator<<(std::ostream &os, const PrivateJointDeterministicDecisionRule<TState, TAction> &decision_rule)
        {
            os << "<private-joint-decision-rule>" << std::endl;
            for (auto individual_decision_rule : decision_rule)
            {
                os << individual_decision_rule << std::endl;
            }
            os << "<private-joint-decision-rule/>" << std::endl;
            return os;
        }

    protected:
        TAction action_agent_i_;
        number agent_id_;
    };

} // namespace sdm

#include <sdm/core/action/private_joint_det_decision_rule.tpp>