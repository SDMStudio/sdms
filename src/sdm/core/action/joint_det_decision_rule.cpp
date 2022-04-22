#include <sdm/core/action/joint_det_decision_rule.hpp>
// #include <sdm/core/joint.hpp>

#include <sdm/core/state/jhistory_tree.hpp>

namespace sdm
{
    JointDeterministicDecisionRule::JointDeterministicDecisionRule() {}

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(const Joint<std::shared_ptr<DecisionRule>> &idr_list, const std::shared_ptr<ActionSpace> &action_space)
    {
        this->joint_idr = idr_list;
        this->action_space = action_space->toDiscreteSpace();
    }

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(std::vector<std::vector<std::shared_ptr<HistoryInterface>>> acc_histories, std::vector<std::vector<std::shared_ptr<Action>>> actions, const std::shared_ptr<ActionSpace> &action_space)
    {
        assert(acc_histories.size() == actions.size());
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        for (std::size_t agent = 0; agent < acc_histories.size(); agent++)
        {
            this->joint_idr.push_back(std::make_shared<DeterministicDecisionRule>(acc_histories[agent], actions[agent]));
        }
    }

    JointDeterministicDecisionRule::JointDeterministicDecisionRule(const std::vector<std::shared_ptr<HistoryInterface>> &, const std::vector<std::shared_ptr<Action>> &list_indiv_dr, const std::shared_ptr<ActionSpace> &action_space)
    {

        auto joint_indiv_dr = list_indiv_dr[0]->to<JointAction>();
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        for (const auto indiv_dr : *joint_indiv_dr)
        {
            this->joint_idr.push_back(indiv_dr->to<DeterministicDecisionRule>());
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
            auto individual_action = this->joint_idr.get(agent)->act(joint_histories.at(agent));
            if (individual_action == nullptr)
                return nullptr;
            joint_action.push_back(individual_action);
        }

        return this->action_space->getItemAddress(joint_action);
    }

    std::shared_ptr<JointAction> JointDeterministicDecisionRule::act(const std::shared_ptr<JointHistoryInterface> &joint_histories) const
    {
        JointAction joint_action(joint_histories->getNumAgents());
        for (number agent = 0; agent < joint_histories->getNumAgents(); agent++)
        {
            auto individual_action = this->joint_idr.get(agent)->act(joint_histories->getIndividualHistory(agent));
            if (individual_action == nullptr)
                return nullptr;
            joint_action[agent] = individual_action;
        }
        return this->action_space->getItemAddress(joint_action)->toJointAction();
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
        assert((this->joint_idr.size() == histories.size()) && (this->joint_idr.size() == actions->size()));

        double probability = 1.;
        for (number agent_id = 0; agent_id < this->joint_idr.size(); agent_id++)
        {
            probability *= this->joint_idr.get(agent_id)->getProbability(histories.at(agent_id), actions->at(agent_id));
        }
        return probability;
    }

    double JointDeterministicDecisionRule::getProbability(const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, const number &agent_id) const
    {
        assert(agent_id < this->joint_idr.size());
        return this->joint_idr.get(agent_id)->getProbability(history, action);
    }

    void JointDeterministicDecisionRule::setProbability(const std::shared_ptr<HistoryInterface> &, const std::shared_ptr<Action> &, double)
    {
        throw exception::Exception("Not implemented exception (JointDeterministicDecisionRule::setProbability)");
    }

    std::size_t JointDeterministicDecisionRule::hash(double precision) const
    {
        size_t seed = 0;
        for (auto &indiv_decision_rule_ptr : this->joint_idr)
        {
            // Combine the hash of the current vector with the hashes of the previous ones
            sdm::hash_combine(seed, indiv_decision_rule_ptr->hash(precision));
        }
        return seed;
    }
    std::string JointDeterministicDecisionRule::str() const
    {
        std::ostringstream res;
        res << "<joint-decision-rule type=\"deterministic\">" << std::endl;
        for (const auto &individual_ddr : this->joint_idr)
        {
            tools::indentedOutput(res, individual_ddr->str().c_str(), 1);
            res << std::endl;
        }
        res << "<joint-decision-rule/>" << std::endl;
        return res.str();
    }

}
