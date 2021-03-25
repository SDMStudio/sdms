#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    SerializedState<TState, TAction>::SerializedState()
    {
    }

    template <typename TState, typename TAction>
    SerializedState<TState, TAction>::SerializedState(TState state) : Pair<TState, std::vector<TAction>>(state, {})
    {
    }
    
    template <typename TState, typename TAction>
    SerializedState<TState, TAction>::SerializedState(TState state, std::vector<TAction> actions) : Pair<TState, std::vector<TAction>>(state, actions)
    {
    }
    template <typename TState, typename TAction>
    SerializedState<TState, TAction>::SerializedState(const SerializedState &v) : Pair<TState, std::vector<TAction>>(v)
    {
    }

    template <typename TState, typename TAction>
    TState SerializedState<TState, TAction>::getState() const 
    {
        return this->first;
    }

    template <typename TState, typename TAction>
    std::vector<TAction> SerializedState<TState, TAction>::getAction()const 
    {
        return this->second;
    }

    template <typename TState, typename TAction>
    number SerializedState<TState, TAction>::getCurrentAgentId() const
    {
        return (this->second).size();
    }

} // namespace sdm