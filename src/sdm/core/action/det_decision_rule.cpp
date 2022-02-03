#include <sdm/core/action/det_decision_rule.hpp>

namespace sdm
{
    DeterministicDecisionRule::DeterministicDecisionRule() {}

    DeterministicDecisionRule::DeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &acc_states, const std::vector<std::shared_ptr<Item>> &n_actions, const std::shared_ptr<Space> &action_space)
    {
        assert(acc_states.size() == n_actions.size());
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        for (std::size_t i = 0; i < acc_states.size(); i++)
        {
            this->setProbability(acc_states[i]->toState(), n_actions[i]->toAction(), 1);
        }
    }

    DeterministicDecisionRule::DeterministicDecisionRule(const std::vector<std::shared_ptr<State>> &acc_states, const std::vector<std::shared_ptr<Action>> &n_actions, const std::shared_ptr<Space> &action_space) 
    {
        if (action_space != nullptr)
            this->action_space = action_space->toDiscreteSpace();
        assert(acc_states.size() == n_actions.size());
        for (std::size_t i = 0; i < acc_states.size(); i++)
        {
            this->setProbability(acc_states[i], n_actions[i], 1);
        }
    }

    DeterministicDecisionRule::DeterministicDecisionRule(const DeterministicDecisionRule &copy) : map_state_to_action_(copy.map_state_to_action_)
    {
        this->action_space = copy.action_space;
    }

    std::shared_ptr<Action> DeterministicDecisionRule::act(const std::shared_ptr<State> &state) const
    {
        auto iter = this->map_state_to_action_.find(state);
        return (iter != this->map_state_to_action_.end()) ? iter->second : nullptr;
    }

    // std::shared_ptr<Action> DeterministicDecisionRule::operator()(const std::shared_ptr<State> &s)
    // {
    //     return this->act(s);
    // }

    double DeterministicDecisionRule::getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const
    {
        return (this->act(state) == action) ? 1 : 0;
    }

    void DeterministicDecisionRule::setProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double proba)
    {
        assert(((proba == 0) || (proba == 1)));
        if (proba == 1)
        {
            this->map_state_to_action_[state] = action;
        }
    }

    std::string DeterministicDecisionRule::str() const
    {
        std::ostringstream res;
        res << "<decision-rule type=\"deterministic\">" << std::endl;
        for (const auto &[state, action] : this->map_state_to_action_)
        {
            res << "\t<decision state=\"" << state->str() << "\" action=\"" << *action << "\"/>" << std::endl;
        }
        res << "<decision-rule/>";
        return res.str();
    }

    std::map<std::shared_ptr<State>, std::shared_ptr<Action>> DeterministicDecisionRule::getMap() const
    {
        return this->map_state_to_action_;
    }

    bool DeterministicDecisionRule::elementExist(const std::shared_ptr<State> &state)
    {
        return (this->map_state_to_action_.find(state) != this->map_state_to_action_.end()) ? true : false;
    }

} // namespace sdm
