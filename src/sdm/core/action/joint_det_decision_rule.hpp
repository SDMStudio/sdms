
#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

namespace sdm
{

    /**
   * @brief The joint deterministic decision rule class. This class is a function that maps joint generic states to joint generic actions. 
   * 
   * @tparam TState the state type
   * @tparam TAction the action type
   */
    template <typename TState, typename TAction>
    class JointDeterministicDecisionRule
        : public Joint<DeterministicDecisionRule<TState, TAction>>,
          public Function<Joint<TState>, Joint<TAction>>
    {
    public:
        using output_type = typename Function<Joint<TState>, Joint<TAction>>::output_type;

        JointDeterministicDecisionRule();
        JointDeterministicDecisionRule(const std::vector<DeterministicDecisionRule<TState, TAction>> &idr_list);
        JointDeterministicDecisionRule(std::vector<std::vector<TState>> acc_states, std::vector<std::vector<TAction>> actions);

        Joint<TAction> act(const Joint<TState> &jobserv) const;
        Joint<TAction> operator()(const Joint<TState> &s);

        friend std::ostream &operator<<(std::ostream &os, const JointDeterministicDecisionRule<TState, TAction> &dr)
        {
            os << "<joint-decision-rule>" << std::endl;
            for (auto idr : dr)
            {
                os << idr << std::endl;
            }
            os << "<joint-decision-rule/>" << std::endl;
            return os;
        }
    };

    template <typename TState, typename TAction>
    using JointDetDecisionRule = JointDeterministicDecisionRule<TState, TAction>;

} // namespace sdm

#include <sdm/core/action/joint_det_decision_rule.tpp>