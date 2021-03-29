#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{

    SerializedState::SerializedState(): Pair<number, std::vector<number>>(0, {})
    {
    }

    SerializedState::SerializedState(number state) : Pair<number, std::vector<number>>(state, {})
    {
    }
    
    SerializedState::SerializedState(number state, std::vector<number> actions) : Pair<number, std::vector<number>>(state, actions)
    {
    }

    SerializedState::SerializedState(const SerializedState &copy) : SerializedState(copy.first, copy.second)
    {
    }

    number SerializedState::getState() const 
    {
        return this->first;
    }

    std::vector<number> SerializedState::getAction()const 
    {
        return this->second;
    }

    number SerializedState::getCurrentAgentId() const
    {
        return (this->second).size();
    }

} // namespace sdm