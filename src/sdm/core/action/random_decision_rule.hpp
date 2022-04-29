
#pragma once

#include <map>

#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/decision_rule.hpp>
#include <sdm/core/space/space.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief A public interface for decision rules. Contains all the methods that must be implemented to well define a decision rule in SDMS.
     */
    class RandomDecisionRule : public DecisionRule
    {
    public:
        RandomDecisionRule(const std::shared_ptr<ActionSpace> &action_space);

        /**
         * @brief Get the action deducted from a given state
         *
         * @param state the generic state
         * @return the corresponding action
         */
        std::shared_ptr<Action> act(const std::shared_ptr<HistoryInterface> &state) const;

        /**
         * @brief Get the probability of action 'action' in state 'state'
         *
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        double getProbability(const std::shared_ptr<HistoryInterface> &state, const std::shared_ptr<Action> &action) const;

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         *
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        void setProbability(const std::shared_ptr<HistoryInterface> &state, const std::shared_ptr<Action> &action, double proba);

        std::string str() const;

    protected:
        std::shared_ptr<ActionSpace> action_space;
    };

} // namespace sdm