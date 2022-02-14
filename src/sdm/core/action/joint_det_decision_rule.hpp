
#pragma once

#include <sdm/core/function.hpp>
#include <sdm/core/action/det_decision_rule.hpp>
#include <sdm/core/joint.hpp>

namespace sdm
{
    /**
     * @brief This class provide a way to manipulate a function that maps joint histories to joint actions.
     */
    class JointDeterministicDecisionRule
        : public DeterministicDecisionRule
    {
    public:
        using input_type = typename DeterministicDecisionRule::input_type;
        using output_type = typename DeterministicDecisionRule::output_type;

        JointDeterministicDecisionRule();
        JointDeterministicDecisionRule(const Joint<std::shared_ptr<DecisionRule>> &individual_decision_rules, const std::shared_ptr<Space> &action_space);
        JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<Item>>> acc_histories, std::vector<std::vector<std::shared_ptr<Item>>> actions, const std::shared_ptr<Space> &action_space);
        JointDeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &, const std::vector<std::shared_ptr<Item>> &list_indiv_dr, const std::shared_ptr<Space> &action_space);

        std::shared_ptr<Action> act(const std::shared_ptr<HistoryInterface> &joint_history) const;
        virtual std::shared_ptr<Action> act(const std::vector<std::shared_ptr<HistoryInterface>> &joint_histories) const;
        virtual std::shared_ptr<JointAction> act(const std::shared_ptr<JointHistoryInterface> &joint_history) const;

        /**
         * @brief Get the probability of selecting action a in history s. This should return 0 if the action that corresponds to the history is a.
         *
         * @param history the history
         * @param action the action
         * @return the probability
         */
        double getProbability(const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action) const;
        virtual double getProbability(const std::vector<std::shared_ptr<HistoryInterface>> &histories, const std::shared_ptr<JointAction> &actions) const;

        /**
         * @brief Get the probability of action 'action' in history 'history' for agent id
         *
         * @param agent_id the agent identifier
         * @param history the history
         * @param action the action
         * @return the probability selecting action 'action' in history 'history'
         */
        double getProbability(const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const number &agent_id) const;

        /**
         * @brief Set the probability of selecting action a in history s.
         *
         * @param s the history
         * @param a the action
         * @param proba the probability
         */
        void setProbability(const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, double proba = 1);

        std::string str() const;

        size_t hash(double precision = 0) const;

        friend std::ostream &operator<<(std::ostream &os, const JointDeterministicDecisionRule &joint_dr)
        {
            os << joint_dr.str();
            return os;
        }

    protected:
        Joint<std::shared_ptr<DecisionRule>> joint_idr;
    };

} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::JointDeterministicDecisionRule>
    {
        inline std::size_t operator()(const sdm::JointDeterministicDecisionRule &in) const
        {
            return in.hash();
        }
    };
}