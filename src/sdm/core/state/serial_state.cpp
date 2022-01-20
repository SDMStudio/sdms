#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

    SerialState::SerialState(): BaseState<Pair<std::shared_ptr<State>, JointAction>>()
    {
    }
    
    SerialState::SerialState(std::shared_ptr<State> state, JointAction actions) : BaseState<Pair<std::shared_ptr<State>, JointAction>>(std::make_pair(state, actions))
    {
        this->setAgentId(actions.size());
    }

    SerialState::SerialState(const SerialState &copy) : BaseState<Pair<std::shared_ptr<State>, JointAction>>(copy)
    {
        this->setAgentId(copy.getCurrentAgentId());
    }

    SerialState::~SerialState() {}

    std::shared_ptr<State> SerialState::getHiddenState() const 
    {
        return this->state.first;
    }

    JointAction SerialState::getAction()const 
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