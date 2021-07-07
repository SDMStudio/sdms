#include <sdm/core/action/joint_det_decision_rule.hpp>
// #include <sdm/core/joint.hpp>

namespace sdm
{
    JointDeterministicDecisionRule::JointDeterministicDecisionRule() {}

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(const Joint<std::shared_ptr<DeterministicDecisionRule>> &idr_list)
        : Joint<std::shared_ptr<DeterministicDecisionRule>>(idr_list)
    {
    }

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<Item>>> acc_states, std::vector<std::vector<std::shared_ptr<Item>>> actions)
    {
        assert(acc_states.size() == actions.size());
        for (std::size_t ag_id = 0; ag_id < acc_states.size(); ag_id++)
        {
            this->push_back(std::make_shared<DeterministicDecisionRule>(acc_states[ag_id], actions[ag_id]));
        }
    }

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &, const std::vector<std::shared_ptr<Item>> &list_indiv_dr)
    {
        auto joint_indiv_dr = list_indiv_dr[0]->to<Joint<std::shared_ptr<Item>>>();
        for (const auto indiv_dr : *joint_indiv_dr)
        {
            this->push_back(indiv_dr->to<DeterministicDecisionRule>());
        }
    }

    std::shared_ptr<Action> JointDeterministicDecisionRule::act(const std::shared_ptr<State> &joint_state) const
    {
        // assert(this->size() == joint_state.size());
        std::shared_ptr<Joint<std::shared_ptr<Action>>> joint_action = std::make_shared<Joint<std::shared_ptr<Action>>>();
        auto joint_state_ = std::static_pointer_cast<Joint<std::shared_ptr<State>>>(joint_state);

        for (number ag_id = 0; ag_id < joint_state_->size(); ag_id++)
        {
            auto indiv_action = this->get(ag_id)->act(joint_state_->get(ag_id));
            joint_action->push_back(indiv_action->toAction());
        }
        return joint_action;
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
            probability *= this->get(agent_id)->getProbability(states.at(agent_id), actions.at(agent_id));
        }
        return probability;
    }

    double JointDeterministicDecisionRule::getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &agent_id) const
    {
        assert(agent_id < this->size());
        return this->get(agent_id)->getProbability(state, action);
    }

    void JointDeterministicDecisionRule::setProbability(const Joint<std::shared_ptr<State>> &states, const Joint<std::shared_ptr<Action>> &actions, double probability)
    {
        assert(probability == 1 || probability == 0);

        if (this->size() == 0)
        {
            assert(states.size() == actions.size());
            for (number agent_id = 0; agent_id < states.size(); agent_id++)
            {
                this->push_back(std::make_shared<DeterministicDecisionRule>());
            }
        }

        assert((this->size() == states.size()) && (this->size() == actions.size()));

        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            (*this)[agent_id]->setProbability(states.at(agent_id), actions.at(agent_id));
        }
    }

    std::string JointDeterministicDecisionRule::str() const
    {
        std::ostringstream res;
        res << "<joint-decision-rule>" << std::endl;
        for (const auto &indiv_dr : *this)
        {
            res << indiv_dr->str() << std::endl;
        }
        res << "<joint-decision-rule/>" << std::endl;
        return res.str();
    }
}

