
#pragma once

#include <map>

#include <sdm/core/action/action.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/space/discrete_space.hpp>

namespace sdm
{
  /**
   * @brief This class provide a way to manipulate data relative to a deterministic decision rule.
   *
   * To represent a deterministic decision rule, we simply use a structure that map histories to actions.
   *
   */
  class DeterministicDecisionRule : public DecisionRule
  {
  public:
    DeterministicDecisionRule();
    DeterministicDecisionRule(const DeterministicDecisionRule &copy);
    DeterministicDecisionRule(const std::vector<std::shared_ptr<Item>> &acc_states, const std::vector<std::shared_ptr<Item>> &n_actions, const std::shared_ptr<Space> &action_space = nullptr);
    DeterministicDecisionRule(const std::vector<std::shared_ptr<HistoryInterface>> &acc_states, const std::vector<std::shared_ptr<Action>> &n_actions, const std::shared_ptr<Space> &action_space = nullptr);

    /**
     * @brief Get the action deducted from a given state
     *
     * @param state the generic state
     * @return the corresponding action
     */
    std::shared_ptr<Action> act(const std::shared_ptr<HistoryInterface> &state) const;

    /**
     * @brief Get the probability of selecting action a in state s. This should return 0 if the action that corresponds to the state is a.
     *
     * @param s the state
     * @param a the action
     * @return the probability
     */
    virtual double getProbability(const std::shared_ptr<HistoryInterface> &s, const std::shared_ptr<Action> &a) const;

    /**
     * @brief Set the probability of selecting action a in state s.
     *
     * @param s the state
     * @param a the action
     * @param proba the probability
     */
    virtual void setProbability(const std::shared_ptr<HistoryInterface> &s, const std::shared_ptr<Action> &a, double proba = 1);

    std::string str() const;

    friend std::ostream &operator<<(std::ostream &os, const DeterministicDecisionRule &dr)
    {
      os << dr.str();
      return os;
    }

    size_t hash(double precision = 0) const;

  protected:
    std::map<std::shared_ptr<HistoryInterface>, std::shared_ptr<Action>> map_history_to_action_;
    std::shared_ptr<DiscreteSpace> action_space;
  };
} // namespace sdm

namespace std
{
  template <>
  struct hash<sdm::DeterministicDecisionRule>
  {
    inline std::size_t operator()(const sdm::DeterministicDecisionRule &in) const
    {
      return in.hash();
    }
  };
}
