#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{

    template <typename TState>
    SerializedState<TState>::SerializedState() : Pair<TState, std::vector<number>>(0, {})
    {
    }

    //A faire !!!! 
    
    template <typename TState>
    SerializedState<TState>::SerializedState(TState size, std::vector<number> default_value) : Pair<TState, std::vector<number>>(size, default_value)
    {
    }
    template <typename TState>
    SerializedState<TState>::SerializedState(const SerializedState &v) : Pair<TState, std::vector<number>>(v)
    {
    }

    template <typename TState>
    TState SerializedState<TState>::getState() const 
    {
        return this->first;
    }

    template <typename TState>
    std::vector<number> SerializedState<TState>::getAction()const 
    {
        return this->second;
    }

    template <typename TState>
    number SerializedState<TState>::getCurrentAgentId() const
    {
        return (this->second).size();
    }

} // namespace sdm