#include <sdm/core/state/serialized_belief_state.hpp>

namespace sdm
{

    SerializedBeliefState::SerializedBeliefState()
    {
        this->agent = 0;
    }

    SerializedBeliefState::SerializedBeliefState(double default_value) : BaseBeliefState<SerializedState>(default_value)
    {
        this->agent = 0;
    }

    SerializedBeliefState::SerializedBeliefState(std::size_t size, double default_value) : BaseBeliefState<SerializedState>(size, default_value)
    {
        this->agent = 0;
    }

    SerializedBeliefState::SerializedBeliefState(const SerializedBeliefState &serial_belief_state) : BaseBeliefState<SerializedState>(serial_belief_state)
    {
        this->agent = serial_belief_state.getCurrentAgentId();
    }

    number SerializedBeliefState::getCurrentAgentId() const
    {
        return this->agent;
    }

    void SerializedBeliefState::setAgent(number agent)
    {
        this->agent = agent;
    }

    typename SerializedBeliefState::state_type::state_type SerializedBeliefState::getHiddenState(const state_type &state) const
    {
        return state.getState();
    }


    std::vector<typename SerializedBeliefState::state_type::action_type> SerializedBeliefState::getAction(const state_type &state) const
    {
        return state.getAction();
    }

} // namespace sdm