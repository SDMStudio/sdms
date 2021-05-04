#include <sdm/core/action/private_joint_det_decision_rule.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    PrivateJointDeterministicDecisionRule<TState, TAction>::PrivateJointDeterministicDecisionRule() {}

    template <typename TState, typename TAction>
    PrivateJointDeterministicDecisionRule<TState, TAction>::PrivateJointDeterministicDecisionRule(const std::vector<DeterministicDecisionRule<TState, TAction>> &idr_list, number action_agent_i, number agent_id)
        : JointDeterministicDecisionRule<TState, TAction>(idr_list), action_agent_i_(action_agent_i), agent_id_(agent_id)
    {
    }

    template <typename TState, typename TAction>
    PrivateJointDeterministicDecisionRule<TState, TAction>::PrivateJointDeterministicDecisionRule(std::vector<std::vector<TState>> acc_states, std::vector<std::vector<TAction>> actions)
    {
        assert(acc_states.size() == actions.size());
        for (std::size_t agent_id = 0; agent_id < acc_states.size(); agent_id++)
        {
            this->push_back(DeterministicDecisionRule<TState, TAction>(acc_states[agent_id], actions[agent_id]));
        }
    }

    template <typename TState, typename TAction>
    Joint<TAction> PrivateJointDeterministicDecisionRule<TState, TAction>::act(const Joint<TState> &jobserv) const
    {
        // assert(this->size() == jobserv.size());
        Joint<TAction> jaction;
        for (number agent_id = 0; agent_id < jobserv.size(); agent_id++)
        {
            if (agent_id == this->agent_id_)
            {
                // Add the action of agent 'agent_id' if 'agent_id' is the agent that owns the private joint decision rule
                jaction.push_back(this->action_agent_i_);
            }
            else
            {
                // Add the decision og agent 'agent_id'
                jaction.push_back(this->at(agent_id).act(jobserv.at(agent_id)));
            }
        }
        return jaction;
    }

    template <typename TState, typename TAction>
    void PrivateJointDeterministicDecisionRule<TState, TAction>::setProbability(const std::vector<TState> &acc_states, const std::vector<TAction> &actions)
    {
        this->push_back(DeterministicDecisionRule<TState, TAction>(acc_states, actions));
    }
}

namespace std
{
    template <typename S, typename A>
    struct hash<sdm::PrivateJointDeterministicDecisionRule<S, A>>
    {
        typedef sdm::PrivateJointDeterministicDecisionRule<S, A> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::JointDeterministicDecisionRule<S, A>>()(in);
        }
    };
}