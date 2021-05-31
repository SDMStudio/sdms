
#pragma once

#include <sdm/core/joint.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

namespace sdm
{
    /**
     * @brief The joint deterministic decision rule class. This class is a function that maps joint generic states to joint generic actions. 
     */
    class JointDeterministicDecisionRules
        : public Joint<DeterministicDecisionRule>,
          public DeterministicDecisionRule
    {
    public:
        JointDeterministicDecisionRule();
        JointDeterministicDecisionRule(const Joint<DeterministicDecisionRule> &individual_decision_rules);
        JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<State>>> acc_states, std::vector<std::vector<std::shared_ptr<Action>>> actions);

        Joint<std::shared_ptr<Action>> act(const Joint<std::shared_ptr<State>> &joint_state) const;

        /**
         * @brief Get the probability of joint action 'action' in joint state 'state'
         * 
         * @param state the joint state
         * @param action the joint action
         * @param proba the probability
         */
        double getProbability(const Joint<std::shared_ptr<State>> &state, const Joint<std::shared_ptr<Action>> &action);

        /**
         * @brief Get the probability of action 'action' in state 'state' for agent id 
         * 
         * @param agent_id the agent identifier
         * @param state the state 
         * @param action the action
         * @return the probability selecting action 'action' in state 'state' 
         */
        double getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &agent_id);

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         * 
         * @param state the joint state
         * @param action the joint action
         * @param proba the probability
         */
        void setProbability(const Joint<std::shared_ptr<State>> &state, const Joint<std::shared_ptr<Action>> &action, double proba = 0);

        std::string str() const
        {
            std::ostringstream res;
            res << "<joint-decision-rule>" << std::endl;
            for (const auto &indiv_dr : *this)
            {
                res << indiv_dr << std::endl;
            }
            res << "<joint-decision-rule/>" << std::endl;
            return res.str();
        }

        friend std::ostream &operator<<(std::ostream &os, const JointDeterministicDecisionRule &joint_dr)
        {
            os << joint_dr.str();
            return os;
        }
    };

} // namespace sdm

#include <sdm/core/action/joint_det_decision_rule.tpp>