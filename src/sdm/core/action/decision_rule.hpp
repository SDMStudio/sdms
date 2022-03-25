
#pragma once

#include <map>

#include <sdm/tools.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/function.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/core/space/discrete_space.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief A public interface for decision rules. Contains all the methods that must be implemented to well define a decision rule in SDMS.
     */
    class DecisionRule : public Action, public Function<std::shared_ptr<HistoryInterface>, std::shared_ptr<Action>>
    {
    public:
        /**
         * @brief Get the action deducted from a given state
         *
         * @param state the generic state
         * @return the corresponding action
         */
        virtual std::shared_ptr<Action> act(const std::shared_ptr<HistoryInterface> &state) const = 0;

        /**
         * @brief Get the action deducted from a given state
         *
         * @param state the generic state
         * @return the corresponding action
         */
        virtual std::shared_ptr<JointAction> act(const std::shared_ptr<JointHistoryInterface> &state) const
        {
            throw sdm::exception::NotImplementedException("NotImplementedException raised in DecisionRule::act");
        }

        /***
         * @brief Apply the DecisionRule function (similar to `act`)
         *
         * @param state the generic states
         * @return the corresponding action
         */
        std::shared_ptr<Action> operator()(const std::shared_ptr<HistoryInterface> &s) { return this->act(s); }

        /**
         * @brief Get the probability of action 'action' in state 'state'
         *
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        virtual double getProbability(const std::shared_ptr<HistoryInterface> &state, const std::shared_ptr<Action> &action) const = 0;

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         *
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        virtual void setProbability(const std::shared_ptr<HistoryInterface> &state, const std::shared_ptr<Action> &action, double proba) = 0;

        virtual std::string str() const = 0;
    };

} // namespace sdm
