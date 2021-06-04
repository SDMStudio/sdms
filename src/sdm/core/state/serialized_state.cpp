#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{

    SerializedState::SerializedState(): BaseState<Pair<std::shared_ptr<State>, Joint<std::shared_ptr<Action>>>>()
    {
    }
    
    SerializedState::SerializedState(std::shared_ptr<State> state, Joint<std::shared_ptr<Action>> actions) : BaseState<Pair<std::shared_ptr<State>, Joint<std::shared_ptr<Action>>>>(std::make_pair(state, actions))
    {
    }

    SerializedState::SerializedState(const SerializedState &copy) : BaseState<Pair<std::shared_ptr<State>, Joint<std::shared_ptr<Action>>>>(copy)
    {
    }

    std::shared_ptr<State> SerializedState::getHiddenState() const 
    {
        return this->state_.first;
    }

    Joint<std::shared_ptr<Action>> SerializedState::getAction()const 
    {
        return this->state_.second;
    }

    number SerializedState::getCurrentAgentId() const
    {
        return this->agentID_;
    }


    void SerializedState::setAgentId(number agentID)
    {
        this->agentID_ = agentID;
    }

} // namespace sdm