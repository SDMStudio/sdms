#include <sdm/core/action/det_decision_rule.hpp>

namespace sdm
{
    DeterministicDecisionRule::DeterministicDecisionRule() {}

    DeterministicDecisionRule::DeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &acc_histories, const std::vector<std::shared_ptr<Item>> &n_actions, const std::shared_ptr<Space> &action_space)
    {
        assert(acc_histories.size() == n_actions.size());
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        for (std::size_t i = 0; i < acc_histories.size(); i++)
        {
            this->setProbability(acc_histories[i]->toState()->toHistory(), n_actions[i]->toAction(), 1);
        }
    }

    DeterministicDecisionRule::DeterministicDecisionRule(const std::vector<std::shared_ptr<HistoryInterface>> &acc_histories, const std::vector<std::shared_ptr<Action>> &n_actions, const std::shared_ptr<Space> &action_space)
    {
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        assert(acc_histories.size() == n_actions.size());
        for (std::size_t i = 0; i < acc_histories.size(); i++)
        {
            this->setProbability(acc_histories[i], n_actions[i], 1);
        }
    }

    DeterministicDecisionRule::DeterministicDecisionRule(const DeterministicDecisionRule &copy) : map_history_to_action_(copy.map_history_to_action_)
    {
        this->action_space = copy.action_space;
    }

    std::shared_ptr<Action> DeterministicDecisionRule::act(const std::shared_ptr<HistoryInterface> &history) const
    {
        auto iter = this->map_history_to_action_.find(history);
        return (iter != this->map_history_to_action_.end()) ? iter->second : nullptr;
    }

    std::shared_ptr<JointAction> DeterministicDecisionRule::act(const std::shared_ptr<JointHistoryInterface> &jhistory) const
    {
        throw sdm::exception::NotImplementedException();
    }

    // std::shared_ptr<Action> DeterministicDecisionRule::operator()(const std::shared_ptr<HistoryInterface> &s)
    // {
    //     return this->act(s);
    // }

    double DeterministicDecisionRule::getProbability(const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action) const
    {
        return (this->act(history) == action) ? 1 : 0;
    }

    void DeterministicDecisionRule::setProbability(const std::shared_ptr<HistoryInterface> &history, const std::shared_ptr<Action> &action, double proba)
    {
        assert(((proba == 0) || (proba == 1)));
        if (proba == 1)
        {
            this->map_history_to_action_[history] = action;
        }
    }

    size_t DeterministicDecisionRule::hash(double precision) const
    {
        size_t seed = 0;
        for (const auto &[history, action] : this->map_history_to_action_)
        {
            sdm::hash_combine(seed, history);
            sdm::hash_combine(seed, action);
        }
        return seed;
    }

    std::string DeterministicDecisionRule::str() const
    {
        std::ostringstream res;
        res << "<decision-rule type=\"deterministic\">" << std::endl;
        for (const auto &[history, action] : this->map_history_to_action_)
        {
            res << "\t<decision history=\"" << history->short_str() << "\" action=\"" << *action << "\"/>" << std::endl;
        }
        res << "<decision-rule/>";
        return res.str();
    }

} // namespace sdm
