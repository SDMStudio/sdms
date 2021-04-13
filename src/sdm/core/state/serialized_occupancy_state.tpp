#include <sdm/core/state/serialized_occupancy_state.hpp>

namespace sdm
{
    
    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState() : OccupancyState<TState,TJointHistory_p>()
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(double default_value) : OccupancyState<TState,TJointHistory_p>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(std::size_t size, double default_value) : OccupancyState<TState,TJointHistory_p>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(const SerializedOccupancyState &v) : OccupancyState<TState,TJointHistory_p>(v)
    {
    }
    /*
    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::jhistory_type> SerializedOccupancyState<TState, TJointHistory_p>::getJointHistories() const
    {
        std::set<jhistory_type> possible_jhistories;
        for (const auto &key : *this)
        {
            possible_jhistories.insert(key.first.second);
        }
        return possible_jhistories;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::state_type> SerializedOccupancyState<TState, TJointHistory_p>::getStates() const
    {
        std::set<state_type> possible_states;
        for (const auto &key : *this)
        {
            possible_states.insert(key.first.first);
        }
        return possible_states;
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> SerializedOccupancyState<TState, TJointHistory_p>::getIndividualHistories(number ag_id) const
    {
        std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::jhistory_type::element_type::ihistory_type> possible_ihistories;
        for (const auto &key : getJointHistories())
        {
            auto ihist = key->getIndividualHistory(ag_id);
            possible_ihistories.insert(ihist);
        }
        return possible_ihistories;
    }*/

    template <typename TState, typename TJointHistory_p>
    number SerializedOccupancyState<TState, TJointHistory_p>::getCurrentAgentId() const
    {
        return this->begin()->first.first.getCurrentAgentId();
    }

    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::state_type> SerializedOccupancyState<TState, TJointHistory_p>::getHiddenStates() const
    {
        std::set<state_type> states = this->getStates();

        std::set<typename state_type::state_type> possible_hidden_states;
        for (const auto &key : states)
        {
            possible_hidden_states.insert(key.getState());
        }
        return possible_hidden_states;

    }


    template <typename TState, typename TJointHistory_p>
    std::set<typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::action_type> SerializedOccupancyState<TState, TJointHistory_p>::getActions() const
    {
        std::set<state_type> states = this->getStates();

        std::set<typename state_type::state_type> possible_action_states;
        for (const auto &key : states)
        {
            possible_action_states.insert(key.getAction());
        }
        return possible_action_states;
    }

    template <typename TState, typename TJointHistory_p>
    typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::state_type SerializedOccupancyState<TState, TJointHistory_p>::getHiddenState(const Pair<state_type,jhistory_type> &state) const
    {
        return state.first.getState();
    }


    template <typename TState, typename TJointHistory_p>
    std::vector<typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::action_type> SerializedOccupancyState<TState, TJointHistory_p>::getAction(const Pair<state_type,jhistory_type> &state) const
    {
        return state.first.getAction();
    }

} // namespace sdm