
#include <sdm/utils/decision_rules/stoch_decision_rule.hpp>
#include <sdm/common.hpp>

namespace sdm
{
    template <typename state_t, typename action_t>
    action_t TabularStochDecisionRule<state_t, action_t>::act(const state_t &state) const
    {
        std::vector<action_t> tmp;
        for (auto elem : this->operator()(state))
            tmp.push_back(elem.second);
        std::discrete_distribution<action_t> distribution(tmp);
        return distribution(sdm::common::global_urng());
    }

    template <typename state_t, typename action_t>
    std::map<action_t, double> TabularStochDecisionRule<state_t, action_t>::operator()(const state_t &state) const
    {
        if (this->find(state) == this->end())
        {
            return std::map<action_t, double>();
        }

        return this->model[state];
    }

    template <typename state_t, typename action_t>
    double TabularStochDecisionRule<state_t, action_t>::operator()(const state_t &state, const action_t &action) const
    {
        return this->getProbability(state, action);
    }

    template <typename state_t, typename action_t>
    double TabularStochDecisionRule<state_t, action_t>::getProbability(state_t state, action_t action) const
    {
        double prob = 0.0;

        if (this->model.find(state) == this->model.end())
        {
            prob = action == default_action ? 1.0 : 0.0;
        }

        else
        {
            prob = this->model.at(state).find(action) != this->model.at(state).end() ? this->model.at(state).at(action) : 0.0;
        }

        return prob;
    }

    template <typename state_t, typename action_t>
    void TabularStochDecisionRule<state_t, action_t>::setProbability(state_t state, action_t action, double prob)
    {
        if (this->find(state) == this->end())
        {
            this->emplace(state, std::map<action_t, double>());
        }

        this->at(state).emplace(action, prob);
    }

} // namespace sdm