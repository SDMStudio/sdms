
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
          public DecisionRule<Joint<TState>, Joint<TAction>>
    {
    public:
        using input_type = TState;
        using output = TAction;
        using output_type = typename Function<Joint<TState>, Joint<TAction>>::output_type;

        JointDeterministicDecisionRule();
        JointDeterministicDecisionRule(const std::vector<DeterministicDecisionRule<TState, TAction>> &individual_decision_rules);
        JointDeterministicDecisionRule(std::vector<std::vector<TState>> acc_states, std::vector<std::vector<TAction>> actions);

        Joint<TAction> act(const Joint<TState> &joint_state) const;

        Joint<TAction> operator()(const Joint<TState> &s);

        /**
         * @brief Get the probability of joint action 'action' in joint state 'state'
         * 
         * @param state the joint state
         * @param action the joint action
         * @param proba the probability
         */
        double getProbability(const Joint<TState> &state, const Joint<TAction> &action);

        /**
         * @brief Get the probability of action 'action' in state 'state' for agent id 
         * 
         * @param agent_id the agent identifier
         * @param state the state 
         * @param action the action
         * @return the probability selecting action 'action' in state 'state' 
         */
        double getProbability(const number &agent_id, const TState &state, const TAction &action);

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         * 
         * @param state the joint state
         * @param action the joint action
         * @param proba the probability
         */
        void setProbability(const Joint<TState> &state, const Joint<TAction> &action, double = 0);

        friend std::ostream &operator<<(std::ostream &os, const JointDeterministicDecisionRule<TState, TAction> &dr)
        {
            os << "<joint-decision-rule>" << std::endl;
            for (const auto &idr : dr)
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