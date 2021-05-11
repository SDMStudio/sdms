
#pragma once

#include <map>

#include <sdm/core/function.hpp>
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
  class DeterministicDecisionRule : public std::map<TState, TAction>, public Function<TState, TAction>
  {
  public:
    using input_type = TState;
    using output = TAction;
    using output_type = typename Function<TState,TAction>::output_type;


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

    friend std::ostream &operator<<(std::ostream &os, const DeterministicDecisionRule<TState, TAction> &dr)
    {
      os << "<decision-rule type=\"deterministic\">" << std::endl;
      for (const auto &pair_s_a : dr)
      {
        os << "\t<decision state=\"" << pair_s_a.first << "\">" << std::endl;
        std::ostringstream res;
        res << "\t\t" << pair_s_a.second << std::endl;
        sdm::tools::indentedOutput(os, res.str().c_str());
        os << "\t<decision/>" << std::endl;
      }
      os << "<decision-rule/>" << std::endl;
      return os;
    }

    /**
     * @brief Add probability 
     * 
     * @param const TState&
     * @param const TAction&
     * 
     */
    void setProbability(const TState& , const TAction&);
  };

  template <typename TState, typename TAction>
  using DetDecisionRule = DeterministicDecisionRule<TState, TAction>;

} // namespace sdm

#include <sdm/core/action/det_decision_rule.tpp>