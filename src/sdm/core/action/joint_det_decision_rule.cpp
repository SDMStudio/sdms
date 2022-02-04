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

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<Item>>> acc_histories, std::vector<std::vector<std::shared_ptr<Item>>> actions, const std::shared_ptr<Space> &action_space)
    {
        assert(acc_histories.size() == actions.size());
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        for (std::size_t agent = 0; agent < acc_histories.size(); agent++)
        {
            this->push_back(std::make_shared<DeterministicDecisionRule>(acc_histories[agent], actions[agent]));
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

    std::shared_ptr<Action> JointDeterministicDecisionRule::act(const std::shared_ptr<HistoryInterface> &joint_histories) const
    {
        return this->act(std::dynamic_pointer_cast<JointHistoryInterface>(joint_histories));
    }

    std::shared_ptr<Action> JointDeterministicDecisionRule::act(const std::vector<std::shared_ptr<HistoryInterface>> &joint_histories) const
    {
        JointAction joint_action;
        for (number agent = 0; agent < joint_histories.size(); agent++)
        {
            auto individual_action = this->get(agent)->act(joint_histories.at(agent));
            if (individual_action == nullptr)
                return nullptr;
            joint_action.push_back(individual_action);
        }

        return this->action_space->getItemAddress(joint_action)->toAction();
    }

    std::shared_ptr<JointAction> JointDeterministicDecisionRule::act(const std::shared_ptr<JointHistoryInterface> &joint_histories) const
    {
        return this->act(joint_histories->getIndividualHistories())->toJointAction();
    }

    double JointDeterministicDecisionRule::getProbability(const std::shared_ptr<HistoryInterface> &jhistories, const std::shared_ptr<Action> &jaction) const
    {
        std::shared_ptr<JointHistoryInterface> joint_histories = std::dynamic_pointer_cast<JointHistoryInterface>(jhistories);
        std::shared_ptr<JointAction> joint_action = std::static_pointer_cast<JointAction>(jaction);


        return this->getProbability(joint_histories->getIndividualHistories(), joint_action);
    }

    // Get probabilities of decision a(u | o)
    double JointDeterministicDecisionRule::getProbability(const std::vector<std::shared_ptr<HistoryInterface>> &histories, const std::shared_ptr<JointAction> &actions) const
    {
        assert((this->size() == histories.size()) && (this->size() == actions->size()));

        double probability = 1.;
        for (number agent_id = 0; agent_id < this->size(); agent_id++)
        {
            probability *= this->get(agent_id)->getProbability(histories.at(agent_id), actions->at(agent_id));
        }
        return probability;
    }

    double JointDeterministicDecisionRule::getProbability(const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const number &agent_id) const
    {
        assert(agent_id < this->size());
        return this->get(agent_id)->getProbability(history, action);
    }

    void JointDeterministicDecisionRule::setProbability(const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &, double)
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

}
