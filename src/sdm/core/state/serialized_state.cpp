#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{

    // template <typename number, typename number>
    SerializedState::SerializedState(): Pair<number, std::vector<number>>(0, {})
    {
    }

    //A faire !!!! 
    
    // template <typename number, typename number>
    SerializedState::SerializedState(number state, std::vector<number> actions) : Pair<number, std::vector<number>>(state, actions)
    {
    }
    // template <typename number, typename number>
    SerializedState::SerializedState(const SerializedState &copy) : SerializedState(copy.first, copy.second)
    {
    }

    // template <typename number, typename number>
    number SerializedState::getState() const 
    {
        return this->first;
    }

    // template <typename number, typename number>
    std::vector<number> SerializedState::getAction()const 
    {
        return this->second;
    }

    // template <typename number, typename number>
    number SerializedState::getCurrentAgentId() const
    {
        return (this->second).size();
    }

} // namespace sdm