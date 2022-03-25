#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

    SerialState::SerialState() : BaseState<Pair<std::shared_ptr<State>, JointAction>>()
    {
    }

    SerialState::SerialState(number num_agents, std::shared_ptr<State> state, JointAction actions) : BaseState<Pair<std::shared_ptr<State>, JointAction>>(std::make_pair(state, actions)), num_agents(num_agents)
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

    JointAction SerialState::getAction() const
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

    number SerialState::getNumAgents() const
    {
        return this->num_agents;
    }

    // double SerialState::getReward(const std::shared_ptr<MDPInterface> &mmdp, const std::shared_ptr<Action> &action, number t)
    // {
    //     // If the agent t is not the last agent, the reward is 0 else this value is the same as the value of the mmdp.
    //     if (!this->isLastAgent(t))
    //     {
    //         return 0;
    //     }
    //     else
    //     {
    //         auto joint_action = this->state.second;
    //         joint_action.push_back(action);

    //         return mmdp->getReward(this->state.first, joint_action, t);
    //     }
    // }

    bool SerialState::operator==(const SerialState &other) const
    {
        return this->isEqual(other);
    }

    std::string SerialState::str() const
    {
        std::ostringstream res;
        res << "SerialState(" << *this->getHiddenState() << ", " << this->getAction() << ")";
        return res.str();
    }

} // namespace sdm