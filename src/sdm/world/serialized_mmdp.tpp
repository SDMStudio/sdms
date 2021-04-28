#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    template <typename TState, typename oAction>
    SerializedMMDP<TState, oAction>::SerializedMMDP(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
    {
        this->setPlanningHorizon(mmdp_->getPlanningHorizon());

        std::vector<std::shared_ptr<DiscreteSpace<TState>>> all_serialized_state;

        number n_agents = this->getNumAgents();

        std::vector<std::vector<oAction>> all_past_action;

        for(int i =0; i<n_agents;i++)
        {
            std::vector<TState> serialized_state_i;
            std::vector<std::vector<oAction>> all_new_action;

            if(i>0)
            {
                for(const auto &action : all_past_action)
                {
                    for(const auto &action_agent_i : this->mmdp_->getActionSpace()->getSpaces()[i-1]->getAll())
                    {
                        std::vector<oAction> temp_action = action;
                        temp_action.push_back(action_agent_i);
                        all_new_action.push_back(temp_action);
                    }
                }
            }else
            {
                all_new_action.push_back({});
            }

            for(const auto &state : this->mmdp_->getStateSpace()->getAll())
            {
                for(const auto &action : all_new_action)
                {
                    serialized_state_i.push_back(TState(state,action));
                }
            }
            all_past_action = all_new_action;
            auto s_i = std::make_shared<DiscreteSpace<TState>>(serialized_state_i);
            all_serialized_state.push_back(s_i);
        }

        this->serialized_state_space_= std::make_shared<MultiSpace<DiscreteSpace<TState>>>(all_serialized_state);
    }

    template <typename TState, typename oAction>
    SerializedMMDP<TState, oAction>::SerializedMMDP(std::string underlying_mmdp) : SerializedMMDP(std::make_shared<DiscreteMMDP>(underlying_mmdp))
    {
    }

    template <typename TState, typename oAction>
    TState SerializedMMDP<TState, oAction>::getInitialState()
    {
        return this->getInternalState();
    }

    template <typename TState, typename oAction>
    TState SerializedMMDP<TState, oAction>::nextState(const TState &serialized_state, const oAction &action, number t, HSVI<TState, oAction> *hsvi) const
    {
        TState new_serialized_state;

        number agent_identifier = serialized_state.getCurrentAgentId();
        auto x = serialized_state.getState();
        auto u = serialized_state.getAction();

        if (agent_identifier +1 != this->getNumAgents())
        {
            u.push_back(action);
            new_serialized_state = TState(x, u);
        }
        else
        {
            double max = std::numeric_limits<double>::min();
            TState smax = 0;
            //u.push_back(action);

            for (const auto &next_serial_state : this->getStateSpaceAt(0)->getAll())
            {
                double tmp = this->getDynamics(serialized_state, action, next_serial_state) * hsvi->do_excess(next_serial_state, t + 1);
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

    template <typename TState, typename oAction>
    double SerializedMMDP<TState, oAction>::getExpectedNextValue(ValueFunction<TState, oAction> *value_function, const TState &serialized_state, const oAction &action, number t) const
    {
        number agent_identifier = serialized_state.getCurrentAgentId();

        auto x = serialized_state.getState();
        auto u = serialized_state.getAction();
        u.push_back(action);

        if (agent_identifier +1!= this->getNumAgents())
        {
            return value_function->getValueAt(TState(x, u), t + 1);
        }
        else
        {
            double tmp = 0;
            for (const auto &next_state : this->getStateSpaceAt(0)->getAll())
            {
                tmp += this->getDynamics(serialized_state, action, next_state) * value_function->getValueAt(next_state, t + 1);
            }
            return tmp;
        }
    }

    template <typename TState, typename oAction>
    double SerializedMMDP<TState, oAction>::getDiscount(number t) const
    {
        if (t % this->getNumAgents() != this->getNumAgents() - 1)
        {
            return 1.0;
        }
        return this->mmdp_->getDiscount();
    }

    template <typename TState, typename oAction>
    void SerializedMMDP<TState, oAction>::setDiscount(double discount)
    {
        this->mmdp_->setDiscount(discount);
        this->discount_ = discount;
    }

    template <typename TState, typename oAction>
    SerializedMMDP<TState,oAction> *SerializedMMDP<TState, oAction>::getUnderlyingProblem()
    {
        return this;
    }

    template <typename TState, typename oAction>
    std::shared_ptr<SerializedMMDP<TState, oAction>> SerializedMMDP<TState, oAction>::getptr()
    {
        return SerializedMMDP<TState, oAction>::shared_from_this();
    }

    template <typename TState, typename oAction>
    std::shared_ptr<SerializedMMDP<TState, oAction>> SerializedMMDP<TState, oAction>::toMDP()
    {
        return this->getptr();
    }

    template <typename TState, typename oAction>
    bool SerializedMMDP<TState, oAction>::isSerialized() const
    {
        return true;
    }
    
    template <typename TState, typename oAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMMDP<TState, oAction>::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    template <typename TState, typename oAction>
    std::shared_ptr<DiscreteSpace<number>> SerializedMMDP<TState,oAction>::getHiddenStateSpace() const
    {
        std::vector<number> all_hidden_state;
        for(const auto &s : this->getStateSpaceAt(0))
        {
            all_hidden_state.push_back(s.getState());
        }

        return std::make_shared<DiscreteSpace<number>>(all_hidden_state);
    }

    template <typename TState, typename oAction>
    std::shared_ptr<DiscreteSpace<SerializedState>> SerializedMMDP<TState,oAction>::getStateSpaceAt(number agent_identifier) const
    {
        return this->serialized_state_space_->getSpace(agent_identifier);
    }

    template <typename TState, typename oAction>
    std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> SerializedMMDP<TState,oAction>::getStateSpace() const
    {
        return this->serialized_state_space_;
    }


    template <typename TState, typename oAction>
    std::shared_ptr<DiscreteSpace<SerializedState>> SerializedMMDP<TState,oAction>::getNextStateSpace(const TState &serialized_state) const
    {
        std::vector<SerializedState> all_state;
        const number agent_identifier = serialized_state.getCurrentAgentId();

        if(agent_identifier +1 == this->getNumAgents())
        {
            return this->getStateSpaceAt(0);
        }else
        {
            for(const auto state : this->getStateSpaceAt(agent_identifier +1)->getAll())
            {
                if(state.getState() == serialized_state.getState())
                {
                    std::vector<number> action = state.getAction();
                    action.pop_back();
                    if(action == serialized_state.getAction())
                    {
                        all_state.push_back(state);
                    }
                }
            }
            return std::make_shared<DiscreteSpace<SerializedState>>(all_state);
        }
    }

    template <typename TState, typename oAction>
    double SerializedMMDP<TState,oAction>::getReward(const TState &s, const oAction &action) const
    {
        if(s.getCurrentAgentId() +1 != this->getNumAgents())
        {
            return 0;
        }else
        {
            std::vector<number> all_action = s.getAction();
            all_action.push_back(action);

            
            return this->mmdp_->getReward(s.getState(),Joint<number>(all_action));
        }
    }

    template <typename TState, typename oAction>
    double SerializedMMDP<TState,oAction>::getReward(const TState &s, const Joint<oAction> &action) const
    {
        if(s.getAction()!= action)
        {
            return 0;
        }else
        {
            return this->mmdp_->getReward(s.getState(),action);
        }
    }

    template <typename TState, typename oAction>
    std::shared_ptr<Reward> SerializedMMDP<TState,oAction>::getReward() const
    {
        return this->mmdp_->getReward();
    }

    template <typename TState, typename oAction>
    std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> SerializedMMDP<TState,oAction>::getObsSpace() const
    {
        return this->getStateSpace();
    }

    template <typename TState, typename oAction>
    std::shared_ptr<DiscreteSpace<SerializedState>> SerializedMMDP<TState,oAction>::getObsSpaceAt(number agent_identifier) const
    {
        return this->getStateSpaceAt(agent_identifier);
    }

    template <typename TState, typename oAction>
    std::shared_ptr<MultiDiscreteSpace<oAction>> SerializedMMDP<TState,oAction>::getActionSpace() const
    {
        return this->mmdp_->getActionSpace();
    }

    // template <typename TState, typename oAction>
    // std::shared_ptr<DiscreteSpace<oAction>> SerializedMMDP<TState,oAction>::getActionSpaceAt(number agent_identifier) const
    // {
    //     return this->mmdp_->getActionSpace()->getSpace(agent_identifier);
    // }

    template <typename TState, typename oAction>
    std::shared_ptr<DiscreteSpace<oAction>> SerializedMMDP<TState,oAction>::getActionSpaceAt(const TState &state) 
    {
        return this->mmdp_->getActionSpace()->getSpace(state.getCurrentAgentId());
    }

    template <typename TState, typename oAction>
    double SerializedMMDP<TState,oAction>::getDynamics(const TState &s,const oAction &action, const TState &s_) const
    {
        if(s.getCurrentAgentId() +1 != this->getNumAgents())
        {
            // If the next serialized_state and the current serialized_state don't have the same hidden or it's not the same player to act, then the dynamics is impossible
            if(s.getCurrentAgentId() +1 != s_.getCurrentAgentId() or s.getState()!= s_.getState())
            {
                return 0;
            }else
            {
                return 1; 
            }
        }else
        {
            std::vector<number> all_action = s.getAction();
            all_action.push_back(action);
            return this->mmdp_->getStateDynamics()->getTransitionProbability(s.getState(),this->getActionSpace()->joint2single(Joint<number>(all_action)),s_.getState());
        }
    }

    template <typename TState, typename oAction>
    number SerializedMMDP<TState,oAction>::getNumAgents() const
    {
        return this->mmdp_->getNumAgents();
    }

    template <typename TState, typename oAction>
    void SerializedMMDP<TState,oAction>::setInternalState(TState new_i_state)
    {
        if(new_i_state.getCurrentAgentId() ==0)
        {
            this->mmdp_->setInternalState(new_i_state.getState());
            this->internal_state_ = new_i_state;
        }
    }

    template <typename TState, typename oAction>
    void SerializedMMDP<TState,oAction>::setPlanningHorizon(number horizon)
    {
        this->mmdp_->setPlanningHorizon(horizon);
        this->planning_horizon_ = horizon;
    }
}