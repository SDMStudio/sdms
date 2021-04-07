#include <sdm/core/state/serialized_belief_state.hpp>

namespace sdm
{

    SerializedBeliefState::SerializedBeliefState()
    {
    }

    SerializedBeliefState::SerializedBeliefState(double default_value) : BaseBeliefState<SerializedState>(default_value)
    {
    }

    SerializedBeliefState::SerializedBeliefState(std::size_t size, double default_value) : BaseBeliefState<SerializedState>(size, default_value)
    {
    }

    SerializedBeliefState::SerializedBeliefState(const SerializedBeliefState &v) : BaseBeliefState<SerializedState>(v)
    {
    }

} // namespace sdm