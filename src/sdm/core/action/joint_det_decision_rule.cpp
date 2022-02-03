#include <sdm/core/action/joint_det_decision_rule.hpp>
// #include <sdm/core/joint.hpp>

#include <sdm/core/state/jhistory_tree.hpp>

namespace sdm
{
    JointDeterministicDecisionRule::JointDeterministicDecisionRule() {}

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(const Joint<std::shared_ptr<DeterministicDecisionRule>> &idr_list, const std::shared_ptr<Space> &action_space)
        : Joint<std::shared_ptr<DeterministicDecisionRule>>(idr_list)
    {
        this->action_space = action_space->toDiscreteSpace();
    }

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<Item>>> acc_states, std::vector<std::vector<std::shared_ptr<Item>>> actions, const std::shared_ptr<Space> &action_space)
    {
        assert(acc_states.size() == actions.size());
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        for (std::size_t agent = 0; agent < acc_states.size(); agent++)
        {
            this->push_back(std::make_shared<DeterministicDecisionRule>(acc_states[agent], actions[agent]));
        }
    }

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &, const std::vector<std::shared_ptr<Item>> &list_indiv_dr, const std::shared_ptr<Space> &action_space)
    {
        auto joint_indiv_dr = list_indiv_dr[0]->to<JointItem>();
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        for (const auto indiv_dr : *joint_indiv_dr)
        {
            this->push_back(indiv_dr->to<DeterministicDecisionRule>());
        }
    }

    std::shared_ptr<Action> JointDeterministicDecisionRule::act(const std::shared_ptr<State> &joint_state) const
    {
        return this->act(std::static_pointer_cast<JointState>(joint_state));
    }

    std::shared_ptr<JointAction> JointDeterministicDecisionRule::act(const std::shared_ptr<JointState> &joint_state) const
    {
        JointAction joint_action;
        for (number agent = 0; agent < joint_state->size(); agent++)
        {
            auto individual_action = this->get(agent)->act(joint_state->get(agent));
            if (individual_action == nullptr)
                return nullptr;
            joint_action.push_back(individual_action);
        }

        return this->action_space->getItemAddress(joint_action)->toAction()->toJointAction();
    }

    // Get probabilities of decision a(u | o)
    double JointDeterministicDecisionRule::getProbability(const JointState &states, const JointAction &actions) const
    {
        assert((this->size() == states.size()) && (this->size() == actions.size()));

        double probability = 1.;
        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            probability *= this->get(agent_id)->getProbability(states.at(agent_id), actions.at(agent_id));
        }
        return probability;
    }

    double JointDeterministicDecisionRule::getProbability(const std::shared_ptr<State> &joint_state, const std::shared_ptr<Action> &joint_action) const
    {
        std::shared_ptr<JointState> joint_state_ = std::static_pointer_cast<JointState>(joint_state);
        std::shared_ptr<JointAction> joint_action_ = std::static_pointer_cast<JointAction>(joint_action);

        double probability = 1.;
        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            probability *= this->get(agent_id)->getProbability(joint_state_->at(agent_id), joint_action_->at(agent_id));
        }
        return probability;
    }

    double JointDeterministicDecisionRule::getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const number &agent_id) const
    {
        assert(agent_id < this->size());
        return this->get(agent_id)->getProbability(state, action);
    }

    void JointDeterministicDecisionRule::setProbability(const JointState &states, const JointAction &actions, double probability)
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

    void JointDeterministicDecisionRule::setProbability(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, double)
    {
        throw exception::Exception("Not implemented exception (JointDeterministicDecisionRule::setProbability)");
    }

    std::string JointDeterministicDecisionRule::str() const
    {
        std::ostringstream res;
        res << "<joint-decision-rule type=\"deterministic\">" << std::endl;
        for (const auto &individual_ddr : *this)
        {
            tools::indentedOutput(res, individual_ddr->str().c_str(), 1);
            res << std::endl;
        }
        res << "<joint-decision-rule/>" << std::endl;
        return res.str();
    }

    bool JointDeterministicDecisionRule::elementExist(const std::shared_ptr<State> &joint_state)
    {
        auto joint_state_ = std::static_pointer_cast<JointState>(joint_state);

        for (number agent = 0; agent < joint_state_->size(); agent++)
        {
            auto individual_decision_rule = this->get(agent);
            auto individual_state = joint_state_->at(agent);

            if (!individual_decision_rule->elementExist(individual_state))
            {
                return false;
            }
        }
        return true;
    }

}
