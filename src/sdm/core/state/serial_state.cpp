#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

    SerialState::SerialState(): BaseState<Pair<std::shared_ptr<State>, Joint<std::shared_ptr<Action>>>>()
    {
    }
    
    SerialState::SerialState(std::shared_ptr<State> state, Joint<std::shared_ptr<Action>> actions) : BaseState<Pair<std::shared_ptr<State>, Joint<std::shared_ptr<Action>>>>(std::make_pair(state, actions))
    {
        this->setAgentId(actions.size());
    }

    SerialState::SerialState(const SerialState &copy) : BaseState<Pair<std::shared_ptr<State>, Joint<std::shared_ptr<Action>>>>(copy)
    {
        this->setAgentId(copy.getCurrentAgentId());
    }

    SerialState::~SerialState() {}

    std::shared_ptr<State> SerialState::getHiddenState() const 
    {
        return this->state.first;
    }

    Joint<std::shared_ptr<Action>> SerialState::getAction()const 
    {
        return this->state.second;
    }

    number SerialState::getCurrentAgentId() const
    {
        return this->agentID_;
    }

    void SerialState::setAgentId(number agentID)
    {
        this->agentID_ = agentID;
    }

    std::string SerialState::str() const
    {
        std::ostringstream res;
        res << "SerialState(" << *this->getHiddenState()<<", "<<this->getAction() << ")";
        return res.str();
    }

} // namespace sdm