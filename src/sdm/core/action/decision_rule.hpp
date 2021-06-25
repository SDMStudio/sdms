
#pragma once

#include <map>

#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    class DecisionRule : public Action, public Function<std::shared_ptr<State>, std::shared_ptr<Action>>
    {
    public:
        /**
         * @brief Get the action deducted from a given state 
         * 
         * @param state the generic state
         * @return the corresponding action
         */
        virtual std::shared_ptr<Action> act(const std::shared_ptr<State> &state) const = 0;

        /***
         * @brief Apply the DecisionRule function (similar to `act`)
         * 
         * @param state the generic states
         * @return the corresponding action
         */
        std::shared_ptr<Action> operator()(const std::shared_ptr<State> &s) { return this->act(s); }

        /**
         * @brief Get the probability of action 'action' in state 'state'
         * 
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        virtual double getProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const = 0;

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         * 
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        virtual void setProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, double proba) = 0;

        virtual std::string str() const = 0;

        virtual TypeAction getTypeAction() const { return TypeAction::DECISION_RULE; }
    };

} // namespace sdm
