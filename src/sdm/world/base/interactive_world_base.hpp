
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
    class InteractiveWorldBase
    {
    protected:
        std::unordered_map<TState, std::unordered_map<TAction, std::discrete_distribution<std::size_t>>> dynamics_generator;
        std::unordered_map<number, std::pair<TState, TObservation>> encoding;

    public:
        void setupDynamicsGenerator();

        TObservation getInitialObservation();
        TObservation nextObservation();
    };
} // namespace sdm