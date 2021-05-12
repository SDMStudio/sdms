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

    // Get probabilities of decision a(u | o)
    template <typename TState, typename TAction>
    double JointDeterministicDecisionRule<TState, TAction>::getProbability(const Joint<TState> &states, const Joint<TAction> &actions)
    {
        assert((this->size() == states.size()) && (this->size() == actions.size()));

        double probability = 1.;
        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            probability *= this->at(agent_id).getProbability(states.at(agent_id), actions.at(agent_id));
        }
        return probability;
    }

    template <typename TState, typename TAction>
    double JointDeterministicDecisionRule<TState, TAction>::getProbability(const number &agent_id, const TState &state, const TAction &action)
    {
        assert(agent_id < this->size());
        return this->at(agent_id).getProbability(state, action);
    }

    template <typename TState, typename TAction>
    void JointDeterministicDecisionRule<TState, TAction>::setProbability(const Joint<TState> &states, const Joint<TAction> &actions, double)
    {
        if (this->size() == 0)
        {
            assert(states.size() == actions.size());
            for (number agent_id = 0; agent_id < states.size(); agent_id++)
            {
                this->push_back(DeterministicDecisionRule<TState, TAction>());
            }
        }
        assert((this->size() == states.size()) && (this->size() == actions.size()));
        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            (*this)[agent_id].setProbability(states.at(agent_id), actions.at(agent_id));
        }
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