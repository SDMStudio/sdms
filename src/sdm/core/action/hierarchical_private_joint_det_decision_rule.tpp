#include <sdm/core/action/hierarchical_private_joint_det_decision_rule.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    HierarchicalPrivateJointDeterministicDecisionRule<TState, TAction>::HierarchicalPrivateJointDeterministicDecisionRule() {}

    template <typename TState, typename TAction>
    HierarchicalPrivateJointDeterministicDecisionRule<TState, TAction>::HierarchicalPrivateJointDeterministicDecisionRule(const std::vector<DeterministicDecisionRule<TState, TAction>> &idr_list, number action_agent_n)
    {   
        this->first = JointDeterministicDecisionRule<TState, TAction>(idr_list);
        this->second = action_agent_n;
        this->n_agents_ = idr_list.size() + 1;
    }

    template <typename TState, typename TAction>
    HierarchicalPrivateJointDeterministicDecisionRule<TState, TAction>::HierarchicalPrivateJointDeterministicDecisionRule(std::vector<std::vector<TState>> acc_states, std::vector<std::vector<TAction>> actions, std::vector<TAction> action_agent_n)
    {
        assert(acc_states.size() == actions.size());
        for (std::size_t agent_id = 0; agent_id < acc_states.size(); agent_id++)
        {
            this->first->push_back(DeterministicDecisionRule<TState, TAction>(acc_states[agent_id], actions[agent_id]));
        }
        this->second = action_agent_n;
        this->n_agents_ = acc_states.size() + 1;
        // this->n_agents_ = acc_states.size();
    }

    template <typename TState, typename TAction>
    Joint<TAction> HierarchicalPrivateJointDeterministicDecisionRule<TState, TAction>::act(const Joint<TState> &jobserv) const
    {   
        // assert(this->size() == jobserv.size());
        Joint<TAction> jaction;
        // For all agents from 1 to N-1:
        for (number agent = 0; agent < this->n_agents_ - 1; agent++)
        {   
            // Get their action using their i. h. history and decision rule. Push it to jaction.
            jaction.push_back(this->first.at(agent).act(jobserv.at(agent)));
        }
        // Push agent N's action.
        jaction.push_back(this->second);
        return jaction;
    }

    template <typename TState, typename TAction>
    void HierarchicalPrivateJointDeterministicDecisionRule<TState, TAction>::setProbability(const std::vector<TState> &acc_states, const std::vector<TAction> &actions)
    {
        this->push_back(DeterministicDecisionRule<TState, TAction>(acc_states, actions));
    }

    template <typename TState, typename TAction>
    number HierarchicalPrivateJointDeterministicDecisionRule<TState, TAction>::getNumAgents() const {
        return this->n_agents_;
    }

}

namespace std
{
    template <typename S, typename A>
    struct hash<sdm::HierarchicalPrivateJointDeterministicDecisionRule<S, A>>
    {
        typedef sdm::HierarchicalPrivateJointDeterministicDecisionRule<S, A> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {   
            size_t seed = 0;
            sdm::hash_combine(seed, in.second);
            for (auto &input : in.first)
            {   
                //Combine the hash of the current vector with the hashes of the previous ones
                sdm::hash_combine(seed, input);
            }
            return seed;
        }
    };
}
