#include <sdm/core/state/serialized_occupancy_state.hpp>

namespace sdm
{

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState() : OccupancyState<TState, TJointHistory_p>()
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(double default_value) : OccupancyState<TState, TJointHistory_p>(default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(std::size_t size, double default_value) : OccupancyState<TState, TJointHistory_p>(size, default_value)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(const SerializedOccupancyState &v) : OccupancyState<TState, TJointHistory_p>(v)
    {
    }

    template <typename TState, typename TJointHistory_p>
    SerializedOccupancyState<TState, TJointHistory_p>::SerializedOccupancyState(const OccupancyState<TState, TJointHistory_p> &v) : OccupancyState<TState, TJointHistory_p>(v)
    {
    }

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
    typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::state_type SerializedOccupancyState<TState, TJointHistory_p>::getHiddenState(const Pair<state_type, jhistory_type> &state) const
    {
        return state.first.getState();
    }

    template <typename TState, typename TJointHistory_p>
    std::vector<typename SerializedOccupancyState<TState, TJointHistory_p>::state_type::action_type> SerializedOccupancyState<TState, TJointHistory_p>::getAction(const Pair<state_type, jhistory_type> &state) const
    {
        return state.first.getAction();
    }
    
    template <typename TState, typename TJointHistory_p>
    std::shared_ptr<SerializedOccupancyState<TState, TJointHistory_p>> SerializedOccupancyState<TState, TJointHistory_p>::getptr()
    {
        return std::static_pointer_cast<SerializedOccupancyState<TState, TJointHistory_p>>(this->shared_from_this());
    }

} // namespace sdm

namespace std
{
    template <typename S, typename V>
    struct hash<sdm::SerializedOccupancyState<S, V>>
    {
        typedef sdm::SerializedOccupancyState<S, V> argument_type;
        typedef std::size_t result_type;
        inline result_type operator()(const argument_type &in) const
        {
            return std::hash<sdm::BaseOccupancyState<S, V>>()(in);
        }
    };
}