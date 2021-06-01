
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
   * @tparam std::shared_ptr<State> the state type
   * @tparam std::shared_ptr<Action> the action type
   */
    class StochasticDecisionRule : public DecisionRule, public RecursiveMap<std::shared_ptr<State>, std::shared_ptr<Action>, double>
    {
    public:
        using input_type = typename DecisionRule::input_type;
        using output_type = typename DecisionRule::output_type;

        StochasticDecisionRule();

        /**
         * @brief Get the action deducted from a given state 
         * 
         * @param s the generic state
         * @return the corresponding action
         */
        std::shared_ptr<Action> act(const std::shared_ptr<State> &s) const;

        // /**
        //  * @brief Apply the DetDecisionRule function (similar to `act` or even `at`)
        //  * 
        //  * @param s the generic state
        //  * @return the corresponding action
        //  */
        // std::shared_ptr<Action> operator()(const std::shared_ptr<State> &s);

        RecursiveMap<std::shared_ptr<Action>, double> getProbabilities(const std::shared_ptr<State> &state) const;

        double getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const;

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         * 
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        void setProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double proba);

        std::string str() const;

        friend std::ostream &operator<<(std::ostream &os, const StochasticDecisionRule &stoch_decision_rule)
        {
            os << stoch_decision_rule.str();
            return os;        
        }
    };

} // namespace sdm

#include <sdm/core/action/stochastic_decision_rule.tpp>