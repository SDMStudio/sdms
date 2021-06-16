#include <sdm/core/action/joint_det_decision_rule.hpp>
// #include <sdm/core/joint.hpp>

namespace sdm
{
    JointDeterministicDecisionRule::JointDeterministicDecisionRule() {}

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(const Joint<DeterministicDecisionRule> &idr_list)
        : Joint<DeterministicDecisionRule>(idr_list)
    {
    }

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<Item>>> acc_states, std::vector<std::vector<std::shared_ptr<Item>>> actions)
    {
        assert(acc_states.size() == actions.size());
        for (std::size_t ag_id = 0; ag_id < acc_states.size(); ag_id++)
        {
            this->push_back(DeterministicDecisionRule(acc_states[ag_id], actions[ag_id]));
        }
    }

    Joint<std::shared_ptr<Action>> JointDeterministicDecisionRule::act(const Joint<std::shared_ptr<State>> &jobserv) const
    {
        // assert(this->size() == jobserv.size());
        Joint<std::shared_ptr<Action>> jaction;
        for (number ag_id = 0; ag_id < jobserv.size(); ag_id++)
        {
            jaction.push_back(this->at(ag_id).act(jobserv.at(ag_id)));
        }
        return jaction;
    }

    // Joint<std::shared_ptr<Action>> JointDeterministicDecisionRule::operator()(const Joint<std::shared_ptr<State>> &s)
    // {
    //     return this->act(s);
    // }

    // Get probabilities of decision a(u | o)
    double JointDeterministicDecisionRule::getProbability(const Joint<std::shared_ptr<State>> &states, const Joint<std::shared_ptr<Action>> &actions) const
    {
        assert((this->size() == states.size()) && (this->size() == actions.size()));

        double probability = 1.;
        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            probability *= this->at(agent_id).getProbability(states.at(agent_id), actions.at(agent_id));
        }
        return probability;
    }

    double JointDeterministicDecisionRule::getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &agent_id) const
    {
        assert(agent_id < this->size());
        return this->at(agent_id).getProbability(state, action);
    }

    void JointDeterministicDecisionRule::setProbability(const Joint<std::shared_ptr<State>> &states, const Joint<std::shared_ptr<Action>> &actions, double probability)
    {
        assert(probability == 1 || probability == 0);

        if (this->size() == 0)
        {
            assert(states.size() == actions.size());
            for (number agent_id = 0; agent_id < states.size(); agent_id++)
            {
                this->push_back(DeterministicDecisionRule());
            }
        }

        assert((this->size() == states.size()) && (this->size() == actions.size()));

        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            (*this)[agent_id].setProbability(states.at(agent_id), actions.at(agent_id));
        }
    }

    std::string JointDeterministicDecisionRule::str() const
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
}

namespace std
{
    // template <typename S, typename A>
    // struct hash<sdm::JointDeterministicDecisionRule<S, A>>
    // {
    //     typedef sdm::JointDeterministicDecisionRule<S, A> argument_type;
    //     typedef std::size_t result_type;
    //     inline result_type operator()(const argument_type &in) const
    //     {
    //         size_t seed = 0;
    //         for (auto &input : in)
    //         {
    //             //Combine the hash of the current vector with the hashes of the previous ones
    //             sdm::hash_combine(seed, input);
    //         }
    //         return seed;
    //     }
    // };
}