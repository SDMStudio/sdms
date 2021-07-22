
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
    DeterministicDecisionRule(const DeterministicDecisionRule &copy);
    DeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &acc_states, const std::vector<std::shared_ptr<Item>> &n_actions);

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
    virtual double getProbability(const std::shared_ptr<State> &s, const std::shared_ptr<Action> &a) const;

    /**
     * @brief Set the probability of selecting action a in state s.
     * 
     * @param s the state
     * @param a the action
     * @param proba the probability
     */
    virtual void setProbability(const std::shared_ptr<State> &s, const std::shared_ptr<Action> &a, double proba = 1);

    std::string str() const;

    friend std::ostream &operator<<(std::ostream &os, const DeterministicDecisionRule &dr)
    {
      os << dr.str();
      return os;
    }

    std::map<std::shared_ptr<State>, std::shared_ptr<Action>> getMap() const;

    bool elementExist(const std::shared_ptr<State>&);

  protected:
    std::map<std::shared_ptr<State>, std::shared_ptr<Action>> map_state_to_action_;
  };
} // namespace sdm

namespace std
{
    template <>
    struct hash<sdm::DeterministicDecisionRule>
    {
        typedef sdm::DeterministicDecisionRule argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            size_t seed = 0;
            for (auto &state_action : in.getMap())
            {
                sdm::hash_combine(seed, state_action.first);
                sdm::hash_combine(seed, state_action.second);
            }
            return seed;
        }
    };
}
