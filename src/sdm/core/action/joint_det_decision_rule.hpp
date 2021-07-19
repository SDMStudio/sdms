
#pragma once

#include <sdm/core/function.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{
    /**
     * @brief The joint deterministic decision rule class. This class is a function that maps joint generic states to joint generic actions. 
     */
    class JointDeterministicDecisionRule
        : public Joint<std::shared_ptr<DeterministicDecisionRule>>,
          public DeterministicDecisionRule
    {
    public:
        using input_type = typename DeterministicDecisionRule::input_type;
        using output_type = typename DeterministicDecisionRule::output_type;

        JointDeterministicDecisionRule();
        JointDeterministicDecisionRule(const Joint<std::shared_ptr<DeterministicDecisionRule>> &individual_decision_rules);
        JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<Item>>> acc_states, std::vector<std::vector<std::shared_ptr<Item>>> actions);
        JointDeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &, const std::vector<std::shared_ptr<Item>> &list_indiv_dr);

        std::shared_ptr<Action> act(const std::shared_ptr<State> &joint_state) const;

        /**
         * @brief Get the probability of joint action 'action' in joint state 'state'
         * 
         * @param state the joint state
         * @param action the joint action
         * @param proba the probability
         */
        double getProbability(const Joint<std::shared_ptr<State>> &state, const Joint<std::shared_ptr<Action>> &action) const;

        /**
         * @brief Get the probability of selecting action a in state s. This should return 0 if the action that corresponds to the state is a.
         * 
         * @param s the state
         * @param a the action
         * @return the probability
         */
        double getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const;

        /**
         * @brief Get the probability of action 'action' in state 'state' for agent id 
         * 
         * @param agent_id the agent identifier
         * @param state the state 
         * @param action the action
         * @return the probability selecting action 'action' in state 'state' 
         */
        double getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &agent_id) const;

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         * 
         * @param state the joint state
         * @param action the joint action
         * @param proba the probability
         */
        void setProbability(const Joint<std::shared_ptr<State>> &state, const Joint<std::shared_ptr<Action>> &action, double proba = 0);

        /**
         * @brief Set the probability of selecting action a in state s.
         * 
         * @param s the state
         * @param a the action
         * @param proba the probability
         */
        void setProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double proba = 1);

        std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, const JointDeterministicDecisionRule &joint_dr)
        {
            os << joint_dr.str();
            return os;
        }
    };

} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::JointDeterministicDecisionRule>
    {
        typedef sdm::JointDeterministicDecisionRule argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (auto &input : in)
            {
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, input);
            }
            return seed;
        }
    };
}