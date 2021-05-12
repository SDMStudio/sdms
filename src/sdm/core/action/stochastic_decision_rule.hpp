
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
   * @brief The stochastic decision rule class. This class is a function that maps generic states distribution over generic actions. 
   * 
   * @tparam TState the state type
   * @tparam TAction the action type
   */
    template <typename TState, typename TAction>
    class StochasticDecisionRule : public DecisionRule<TState, TAction>, public RecursiveMap<TState, TAction, double>
    {
    public:
        using input_type = typename DecisionRule<TState, TAction>::input_type;
        using output_type = typename DecisionRule<TState, TAction>::output_type;

        StochasticDecisionRule();

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

        RecursiveMap<TAction, double> getProbabilities(const TState &state);

        double getProbability(const TState &state, const TAction &action);

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         * 
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        void setProbability(const TState &state, const TAction &action, double proba);

        friend std::ostream &operator<<(std::ostream &os, const StochasticDecisionRule<TState, TAction> &stoch_decision_rule)
        {
            os << "<decision-rule type=\"stochastic\">" << std::endl;
            for (const auto &pair_state__pair_action__proba : stoch_decision_rule)
            {
                os << "\t<decision state=\"" << pair_state__pair_action__proba.first << "\">" << std::endl;
                std::ostringstream res;
                res << "\t\t" << pair_state__pair_action__proba.second << std::endl;
                sdm::tools::indentedOutput(os, res.str().c_str());
                os << "\t<decision/>" << std::endl;
            }
            os << "<decision-rule/>" << std::endl;
            return os;
        }
    };

    template <typename TState, typename TAction>
    using StochDecisionRule = StochasticDecisionRule<TState, TAction>;

} // namespace sdm

#include <sdm/core/action/stochastic_decision_rule.tpp>