
#pragma once

#include <sdm/types.hpp>
#include <sdm/spaces.hpp>
#include <sdm/core/reward.hpp>
#include <sdm/algorithms/hsvi.hpp>
#include <sdm/utils/value_function/value_function.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    template <typename TState, typename TAction>
    class SolvableByHSVI
    {
    public:
        /**
         * @brief Get the initial state
         */
        virtual TState &getInitialState() = 0;

        virtual TState nextState(TState state, TAction action, int t = 0, HSVI<TState, TAction> *hsvi = nullptr) const = 0;
        virtual double getDiscount() = 0;
        virtual Reward getReward() = 0;
        virtual void setDiscount(double discount) = 0;
        virtual DiscreteSpace<TAction> getActionSpace(TState state) = 0;
        virtual double getReward(TState state, TAction action) const = 0;
        virtual double getExpectedNextValue(ValueFunction<TState, TAction> *value_function, TState state, TAction action, int t = 0) const = 0;
    };
} // namespace sdm