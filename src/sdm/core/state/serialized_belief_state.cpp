#include <sdm/core/state/serialized_belief_state.hpp>

namespace sdm
{

    SerializedBeliefState::SerializedBeliefState()
    {
    }

    SerializedBeliefState::SerializedBeliefState(double default_value) : BaseBeliefState<SerializedState>(default_value)
    {
    }

    // SerializedBeliefState::SerializedBeliefState(std::size_t size, double default_value) : BaseBeliefState<SerializedState>(size, default_value)
    // {
    // }

    SerializedBeliefState::SerializedBeliefState(const SerializedBeliefState &v) : BaseBeliefState<SerializedState>(v)
    {
    }

    number SerializedBeliefState::getCurrentAgentId() const
    {
        return this->begin()->first.getCurrentAgentId();
    }

    // std::set<typename SerializedBeliefState::state_type::state_type> SerializedBeliefState::getHiddenStates() const
    // {
    //     // std::set<state_type> states = this->getStates();

    //     std::set<typename state_type::state_type> possible_hidden_states;
    //     // for (const auto &key : states)
    //     // {
    //     //     possible_hidden_states.insert(key.getState());
    //     // }
    //     return possible_hidden_states;

    // }

    // std::set<typename SerializedBeliefState::state_type::action_type> SerializedBeliefState::getActions() const
    // {
    //     // std::set<state_type> states = this->getStates();

    //     std::set<typename state_type::state_type> possible_action_states;
    //     // for (const auto &key : states)
    //     // {
    //     //     possible_action_states.insert(key.getAction());
    //     // }
    //     return possible_action_states;
    // }

    typename SerializedBeliefState::state_type::state_type SerializedBeliefState::getHiddenState(const state_type &state) const
    {
        return state.getState();
    }


    std::vector<typename SerializedBeliefState::state_type::action_type> SerializedBeliefState::getAction(const state_type &state) const
    {
        return state.getAction();
    }

} // namespace sdm