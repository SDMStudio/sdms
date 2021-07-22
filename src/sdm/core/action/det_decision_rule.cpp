#include <sdm/core/action/det_decision_rule.hpp>

namespace sdm
{
    DeterministicDecisionRule::DeterministicDecisionRule() {}

    DeterministicDecisionRule::DeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &acc_states, const std::vector<std::shared_ptr<Item>> &n_actions)
    {
        assert(acc_states.size() == n_actions.size());
        for (std::size_t i = 0; i < acc_states.size(); i++)
        {
            this->setProbability(acc_states[i]->toState(), n_actions[i]->toAction(), 1);
        }
    }

    DeterministicDecisionRule::DeterministicDecisionRule(const DeterministicDecisionRule &copy) : map_state_to_action_(copy.map_state_to_action_)
    {
    }

    std::shared_ptr<Action> DeterministicDecisionRule::act(const std::shared_ptr<State> &state) const
    {
        try
        {

            return this->map_state_to_action_.at(state);
        }
        catch (const std::exception &e)
        {
            std::cerr << "State not found in the DeterministicDecisionRule, error :" << e.what() << '\n';
            exit(-1);
        }
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
        for (const auto &state_action : this->map_state_to_action_)
        {
            res << "\t<decision state=\"" << state_action.first->str() << "\" action=\"" << *state_action.second << "\"/>" << std::endl;
        }
        res << "<decision-rule/>";
        return res.str();
    }

    std::map<std::shared_ptr<State>, std::shared_ptr<Action>> DeterministicDecisionRule::getMap() const
    {
        return this->map_state_to_action_;
    }

    bool DeterministicDecisionRule::elementExist(const std::shared_ptr<State>& state)
    {
        return (this->map_state_to_action_.find(state) != this->map_state_to_action_.end()) ? true : false;
    }


} // namespace sdm

