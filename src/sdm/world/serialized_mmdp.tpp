#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
    SerializedMMDP::SerializedMMDP()
    {
    }

    SerializedMMDP::SerializedMMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : serial_mmdp_(std::make_shared<SerializedMMDPStructure>(underlying_mmdp))
    {
    }

    SerializedMMDP::SerializedMMDP(std::string underlying_mmdp) : SerializedMMDP(std::make_shared<DiscreteMMDP>(underlying_mmdp))
    {
    }

    SerializedState SerializedMMDP::getInitialState()
    {
        return this->serial_mmdp_->getInternalState();
    }

    SerializedState SerializedMMDP::nextState(const SerializedState &serialized_state, const number &action, number t, std::shared_ptr<HSVI<SerializedState, number>> hsvi) const
    {
        SerializedState new_serialized_state;

        number agent_identifier = serialized_state.getCurrentAgentId();
        auto x = serialized_state.getState();
        auto u = serialized_state.getAction();

        if (agent_identifier +1 != this->serial_mmdp_->getNumAgents())
        {
            u.push_back(action);
            new_serialized_state = SerializedState(x, u);
        }
        else
        {
            SerializedState smax = 0;
            double max = std::numeric_limits<double>::min();

            for (const auto &next_serial_state : this->serial_mmdp_->getReachableSerialStates(serialized_state, action))
            {
                double tmp = this->serial_mmdp_->getTransitionProbability(serialized_state, action, next_serial_state) * hsvi->do_excess(next_serial_state, t + 1);
                if (tmp > max)
                {
                    max = tmp;
                    smax = next_serial_state;
                }
            }
            new_serialized_state = smax;
        }
        return new_serialized_state;
    }

    double SerializedMMDP::getExpectedNextValue(std::shared_ptr<ValueFunction<SerializedState, number>> value_function, const SerializedState &serialized_state, const number &action, number t) const
    {
        number agent_identifier = serialized_state.getCurrentAgentId();

        auto x = serialized_state.getState();
        auto u = serialized_state.getAction();
        u.push_back(action);

        if (agent_identifier + 1 != this->serial_mmdp_->getNumAgents())
        {
            return value_function->getValueAt(SerializedState(x, u), t + 1);
        }
        else
        {
            double tmp = 0;
            for (const auto &next_serial_state : this->serial_mmdp_->getReachableSerialStates(serialized_state, action))
            {
                tmp += this->serial_mmdp_->getTransitionProbability(serialized_state, action, next_serial_state) * value_function->getValueAt(next_serial_state, t + 1);
            }
            return tmp;
        }
    }

    SerializedMMDPStructure *SerializedMMDP::getUnderlyingProblem()
    {
        return this->serial_mmdp_.get();
    }

    std::shared_ptr<SerializedMMDP> SerializedMMDP::getptr()
    {
        return SerializedMMDP::shared_from_this();
    }

    std::shared_ptr<SerializedMMDP> SerializedMMDP::toMDP()
    {
        return this->getptr();
    }

    bool SerializedMMDP::isSerialized() const
    {
        return true;
    }
    
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMMDP::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    double SerializedMMDP::getReward(const SerializedState &s, const number &action) const
    {
        return this->serial_mmdp_->getReward(s,action);
    }

    std::shared_ptr<DiscreteSpace<number>> SerializedMMDP::getActionSpaceAt(const SerializedState &state) 
    {
        return this->serial_mmdp_->getActionSpace()->getSpace(state.getCurrentAgentId());
    }

}