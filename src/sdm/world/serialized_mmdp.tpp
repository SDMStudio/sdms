#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    SerializedMMDP<TState, TAction>::SerializedMMDP()
    {
    }

    template <typename TState, typename TAction>
    SerializedMMDP<TState, TAction>::SerializedMMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : serial_mmdp_(std::make_shared<SerializedMMDPStructure<TState,number>>(underlying_mmdp))
    {
    }

    template <typename TState, typename TAction>
    SerializedMMDP<TState, TAction>::SerializedMMDP(std::string underlying_mmdp) : SerializedMMDP(std::make_shared<DiscreteMMDP>(underlying_mmdp))
    {
    }

    template <typename TState, typename TAction>
    TState SerializedMMDP<TState, TAction>::getInitialState()
    {
        return this->serial_mmdp_->getInternalState();
    }

    template <typename TState, typename TAction>
    TState SerializedMMDP<TState, TAction>::nextState(const TState &serialized_state, const TAction &action, number t, std::shared_ptr<HSVI<TState, TAction>> hsvi) const
    {
        TState new_serialized_state;

        number agent_identifier = serialized_state.getCurrentAgentId();
        auto x = serialized_state.getState();
        auto u = serialized_state.getAction();

        if (agent_identifier +1 != this->serial_mmdp_->getNumAgents())
        {
            u.push_back(action);
            new_serialized_state = TState(x, u);
        }
        else
        {
            TState smax = 0;
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

    template <typename TState, typename TAction>
    double SerializedMMDP<TState, TAction>::getExpectedNextValue(ValueFunction<TState, TAction> *value_function, const TState &serialized_state, const TAction &action, number t) const
    {
        number agent_identifier = serialized_state.getCurrentAgentId();

        auto x = serialized_state.getState();
        auto u = serialized_state.getAction();
        u.push_back(action);

        if (agent_identifier + 1 != this->serial_mmdp_->getNumAgents())
        {
            return value_function->getValueAt(TState(x, u), t + 1);
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

    template <typename TState, typename TAction>
    SerializedMMDPStructure<TState,TAction> *SerializedMMDP<TState, TAction>::getUnderlyingProblem()
    {
        return this->serial_mmdp_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDP<TState, TAction>> SerializedMMDP<TState, TAction>::getptr()
    {
        return SerializedMMDP<TState, TAction>::shared_from_this();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDP<TState, TAction>> SerializedMMDP<TState, TAction>::toMDP()
    {
        return this->getptr();
    }

    template <typename TState, typename TAction>
    bool SerializedMMDP<TState, TAction>::isSerialized() const
    {
        return true;
    }
    
    template <typename TState, typename TAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMMDP<TState, TAction>::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename TAction>
    double SerializedMMDP<TState,TAction>::getReward(const TState &s, const TAction &action) const
    {
        return this->serial_mmdp_->getReward(s,action);
        // if(s.getCurrentAgentId() +1 != this->serial_mmdp_->getNumAgents())
        // {
        //     return 0;
        // }else
        // {
        //     std::vector<number> all_action = s.getAction();
        //     all_action.push_back(action);

            
        //     return this->serial_mmdp_->getReward(s.getState(),Joint<number>(all_action));
        // }
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<TAction>> SerializedMMDP<TState,TAction>::getActionSpaceAt(const TState &state) 
    {
        return this->serial_mmdp_->getActionSpace()->getSpace(state.getCurrentAgentId());
    }

}