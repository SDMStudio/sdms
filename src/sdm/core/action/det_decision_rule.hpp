
#pragma once

#include <map>

#include <sdm/core/action/decision_rule.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/linear_algebra/mapped_vector.hpp>
#include <sdm/tools.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{

  /**
   * @brief The deterministic decision rule class. This class is a function that maps generic states to generic actions. 
   * 
   * @tparam TState the state type
   * @tparam TAction the action type
   */
  template <typename TState, typename TAction>
  class DeterministicDecisionRule : public DecisionRule<TState, TAction>, public std::map<TState, TAction>
  {
  public:
    using input_type = typename DecisionRule<TState, TAction>::input_type;
    using output_type = typename DecisionRule<TState, TAction>::output_type;
    using output = TAction;

    DeterministicDecisionRule();
    DeterministicDecisionRule(std::vector<TState> acc_states, std::vector<TAction> n_actions);

    /**
     * @brief Get the action deducted from a given state 
     * 
     * @param s the generic state
     * @return the corresponding action
     */
    TAction act(const TState &s) const;

    /**
     * @brief Apply the DetDecisionRule function (similar to `act` or even `at`)
     * 
     * @param s the generic state
     * @return the corresponding action
     */
    TAction operator()(const TState &s);

    double getProbability(const TState &state, const TAction &action);

    /**
     * @brief Add probability 
     * 
     * @param const TState&
     * @param const TAction&
     * 
     */
    void setProbability(const TState &, const TAction &, double = 0);

    friend std::ostream &operator<<(std::ostream &os, const DeterministicDecisionRule<TState, TAction> &dr)
    {
      os << "<decision-rule type=\"deterministic\">" << std::endl;
      for (const auto &pair_s_a : dr)
      {
        os << "\t<decision state=\"" << pair_s_a.first->short_str() << "\" action=\"" << pair_s_a.second << "\"/>" << std::endl;
      }
      os << "<decision-rule/>" << std::endl;
      return os;
    }
  };

  template <typename TState, typename TAction>
  using DetDecisionRule = DeterministicDecisionRule<TState, TAction>;

} // namespace sdm

#include <sdm/core/action/det_decision_rule.tpp>