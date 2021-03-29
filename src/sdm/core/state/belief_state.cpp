#include <sdm/core/state/belief_state.hpp>

namespace sdm
{
    BeliefState::BeliefState()
    {
    }

    BeliefState::BeliefState(double default_value) : MappedVector<number, double>(default_value)
    {
    }

    BeliefState::BeliefState(std::size_t size, double default_value) : MappedVector<number, double>(size, default_value)
    {
    }

    BeliefState::BeliefState(const BeliefState &v) : MappedVector<number, double>(v)
    {
    }

    number BeliefState::getState(const number &state)
    {
        return state;
    }

} // namespace sdm
