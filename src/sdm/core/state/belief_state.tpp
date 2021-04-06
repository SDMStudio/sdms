#include <sdm/core/state/belief_state.hpp>

namespace sdm
{
    template <typename TState>
    BaseBeliefState<TState>::BaseBeliefState()
    {
    }

    template <typename TState>
    BaseBeliefState<TState>::BaseBeliefState(double default_value) : MappedVector<TState, double>(default_value)
    {
    }

    template <typename TState>
    BaseBeliefState<TState>::BaseBeliefState(std::size_t size, double default_value) : MappedVector<TState, double>(size, default_value)
    {
    }

    template <typename TState>
    BaseBeliefState<TState>::BaseBeliefState(const BaseBeliefState &v) : MappedVector<TState, double>(v)
    {
    }

    template <typename TState>
    TState BaseBeliefState<TState>::getState(const TState &state)
    {
        return state;
    }

} // namespace sdm
