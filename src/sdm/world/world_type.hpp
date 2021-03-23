#pragma once

#include <sdm/types.hpp>
#include <sdm/core/states.hpp>

namespace sdm
{

    class DiscreteMDP;
    class DiscretePOMDP;
    class DiscreteDecPOMDP;

    template <typename TState, typename TAction>
    struct WorldType
    {
        template <bool is_solving_mdp = std::is_same<TState, number>::value>
        static std::enable_if_t<is_solving_mdp, DiscreteMDP>
        getAssociatedProblem();

        template <bool is_solving_pomdp = std::is_same<TState, BeliefState>::value>
        static std::enable_if_t<is_solving_pomdp, DiscretePOMDP>
        getAssociatedProblem();
        
        template <bool is_solving_decpomdp = std::is_any<TState, OccupancyState<>, SerializedOccupancyState<>>::value>
        static std::enable_if_t<is_solving_decpomdp, DiscreteDecPOMDP>
        getAssociatedProblem();

        using type = decltype(WorldType<TState, TAction>::getAssociatedProblem());
    };
} // namespace sdm