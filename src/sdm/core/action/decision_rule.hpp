
#pragma once

#include <map>

#include <sdm/tools.hpp>
#include <sdm/core/function.hpp>

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
    class DecisionRule : public Function<TState, TAction>
    {
    public:
        using input_type = typename Function<TState, TAction>::input_type;
        using output_type = typename Function<TState, TAction>::output_type;

        /**
         * @brief Selects an action given an input (state, observation, history, etc).
         */
        virtual TAction operator()(const TState &s) = 0;

        /**
         * @brief Get the probability of action 'action' in state 'state'
         * 
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        virtual double getProbability(const TState &state, const TAction &action) = 0;

        /**
         * @brief Sets the probability of selecting action a when observing state s.
         * 
         * @param state the state
         * @param action the action
         * @param proba the probability
         */
        virtual void setProbability(const TState &state, const TAction &action, double proba) = 0;
    };

} // namespace sdm
