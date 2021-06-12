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
        for (const auto &pair_s_a : this->map_state_to_action_)
        {
            res << "\t<decision state=\"" << pair_s_a.first->str() << "\" action=\"" << pair_s_a.second->str() << "\" probability=\"" << this->getProbability(pair_s_a.first, pair_s_a.second) << "/>" << std::endl;
        }
        res << "<decision-rule/>" << std::endl;
        return res.str();
    }

} // namespace sdm

namespace std
{
    // template <>
    // struct hash<sdm::DeterministicDecisionRule>
    // {
    //     typedef sdm::DeterministicDecisionRule argument_type;
    //     typedef std::size_t result_type;
    //     inline result_type operator()(const argument_type &in) const
    //     {
    //         size_t seed = 0;
    //         for (auto &input : in)
    //         {
    //             sdm::hash_combine(seed, input.first);
    //             sdm::hash_combine(seed, input.second);
    //         }
    //         return seed;
    //     }
    // };
}