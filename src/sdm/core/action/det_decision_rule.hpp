
#pragma once

#include <map>

#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/action/decision_rule.hpp>

namespace sdm
{
  class DeterministicDecisionRule : public DecisionRule
  {
  public:
    DeterministicDecisionRule();
    DeterministicDecisionRule(const std::vector<std::shared_ptr<State>> &acc_states, const std::vector<std::shared_ptr<Action>> &n_actions);

    /**
     * @brief Get the action deducted from a given state 
     * 
     * @param state the generic state
     * @return the corresponding action
     */
    std::shared_ptr<Action> act(const std::shared_ptr<State> &state) const;

    /**
     * @brief Get the probability of selecting action a in state s. This should return 0 if the action that corresponds to the state is a.
     * 
     * @param s the state
     * @param a the action
     * @return the probability
     */
    virtual double getProbability(const std::shared_ptr<State> &s, const std::shared_ptr<Action> &a);

    /**
     * @brief Set the probability of selecting action a in state s.
     * 
     * @param s the state
     * @param a the action
     * @param proba the probability
     */
    virtual void setProbability(const std::shared_ptr<State> &s, const std::shared_ptr<Action> &a, double proba = 0);

    std::string str() const
    {
      std::ostringstream res;
      res << "<decision-rule type=\"deterministic\">" << std::endl;
      for (const auto &pair_s_a : this->map_state_to_action_)
      {
        os << "\t<decision state=\"" << pair_s_a.first->str() << "\" action=\"" << pair_s_a.second->str() << "\"/>" << std::endl;
      }
      os << "<decision-rule/>" << std::endl;
      return res.str();
    }

    friend std::ostream &operator<<(std::ostream &os, const DeterministicDecisionRule &dr)
    {
      os << dr.str();
      return os;
    }

  protected:
    std::map<std::shared_ptr<State>, std::shared_ptr<Action>> map_state_to_action_;
  };
} // namespace sdm

#include <sdm/core/action/det_decision_rule.tpp>