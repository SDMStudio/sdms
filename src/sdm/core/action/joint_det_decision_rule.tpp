#include <sdm/core/action/joint_det_decision_rule.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    JointDeterministicDecisionRule<TState, TAction>::JointDeterministicDecisionRule() {}

    template <typename TState, typename TAction>
    JointDeterministicDecisionRule<TState, TAction>::JointDeterministicDecisionRule(const std::vector<DeterministicDecisionRule<TState, TAction>> &idr_list)
        : Joint<DeterministicDecisionRule<TState, TAction>>(idr_list)
    {
    }

    template <typename TState, typename TAction>
    JointDeterministicDecisionRule<TState, TAction>::JointDeterministicDecisionRule(std::vector<std::vector<TState>> acc_states, std::vector<std::vector<TAction>> actions)
    {
        assert(acc_states.size() == actions.size());
        for (std::size_t ag_id = 0; ag_id < acc_states.size(); ag_id++)
        {
            this->push_back(DeterministicDecisionRule<TState, TAction>(acc_states[ag_id], actions[ag_id]));
        }
    }

    template <typename TState, typename TAction>
    Joint<TAction> JointDeterministicDecisionRule<TState, TAction>::act(const Joint<TState> &jobserv) const
    {
        // assert(this->size() == jobserv.size());
        Joint<TAction> jaction;
        for (number ag_id = 0; ag_id < jobserv.size(); ag_id++)
        {
            jaction.push_back(this->at(ag_id).act(jobserv.at(ag_id)));
        }
        return jaction;
    }

    template <typename TState, typename TAction>
    Joint<TAction> JointDeterministicDecisionRule<TState, TAction>::operator()(const Joint<TState> &s)
    {
        return this->act(s);
    }

    template <typename TState, typename TAction>
    void JointDeterministicDecisionRule<TState, TAction>::setProbability(const std::vector<TState>& acc_states , const std::vector<TAction>& actions)
    {
        this->push_back(DeterministicDecisionRule<TState, TAction>(acc_states, actions));
    }
}


namespace std
{
    template <typename S, typename A>
    struct hash<sdm::JointDeterministicDecisionRule<S, A>>
    {
        typedef sdm::JointDeterministicDecisionRule<S, A> argument_type;
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