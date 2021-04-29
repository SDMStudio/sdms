#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
    template <typename TState, typename TAction>
    SerializedMMDPStructure<TState, TAction>::SerializedMMDPStructure()
    {
    }

    template <typename TState, typename TAction>
    SerializedMMDPStructure<TState, TAction>::SerializedMMDPStructure(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
    {
        this->setPlanningHorizon(mmdp_->getPlanningHorizon());
        this->setDiscount(mmdp_->getDiscount());
        this->setActionSpace(mmdp_->getActionSpace());

        this->createInitSerializedStateSpace();
        this->createInitReachableStateSpace();
    }

    template <typename TState, typename TAction>
    SerializedMMDPStructure<TState, TAction>::SerializedMMDPStructure(std::string underlying_mmdp) : SerializedMMDPStructure(std::make_shared<DiscreteMMDP>(underlying_mmdp))
    {
    }

    template <typename TState, typename TAction>
    void SerializedMMDPStructure<TState, TAction>::createInitSerializedStateSpace()
    {
        std::vector<std::shared_ptr<DiscreteSpace<TState>>> all_serialized_state;

        number n_agents = this->getNumAgents();

        std::vector<std::vector<TAction>> all_past_action;

        for(int i =0; i<n_agents;i++)
        {
            std::vector<TState> serialized_state_i;
            std::vector<std::vector<TAction>> all_new_action;

            if(i>0)
            {
                for(const auto &action : all_past_action)
                {
                    for(const auto &action_agent_i : this->mmdp_->getActionSpace()->getSpaces()[i-1]->getAll())
                    {
                        std::vector<TAction> temp_action = action;
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

    template <typename TState, typename TAction>
    void SerializedMMDPStructure<TState, TAction>::createInitReachableStateSpace()
    {
        for(const auto serialized_state : this->serialized_state_space_->getAll())
        {
            auto x = serialized_state.getState();
            auto u = serialized_state.getAction();
            number agent_identifier = serialized_state.getCurrentAgentId();

            std::unordered_map<TAction,std::set<TState>> map_action_next_serial_state;

            for(auto action : this->getActionSpace()->getSpace(agent_identifier)->getAll())
            {
                std::vector<TAction> serial_action(u);
                serial_action.push_back(action);

                std::set<TState> all_next_serial_state;

                if(agent_identifier +1 == this->getNumAgents())
                {
                    Joint<TAction> joint_action(serial_action);
                    try
                    {
                        for(const auto next_state : this->mmdp_->getReachableStates(x, joint_action))
                        {
                            all_next_serial_state.insert(SerializedState(next_state,std::vector<TAction>()));
                        }
                    }
                    catch(const std::exception& e)
                    {
                    }
                }else
                {

                    all_next_serial_state.insert(SerializedState(x,u));
                }
                map_action_next_serial_state.emplace(action,all_next_serial_state);
            }
            this->reachable_state_space.emplace(serialized_state,map_action_next_serial_state);
        }
    }

    template <typename TState, typename TAction>
    double SerializedMMDPStructure<TState, TAction>::getDiscount(number t) const
    {
        return (t % this->getNumAgents() == this->getNumAgents() - 1) ? this->mmdp_->getDiscount() : 1.0;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDPStructure<TState, TAction>> SerializedMMDPStructure<TState, TAction>::getptr()
    {
        return SerializedMMDPStructure<TState, TAction>::shared_from_this();
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDPStructure<TState, TAction>> SerializedMMDPStructure<TState, TAction>::toMDP()
    {
        return this->getptr();
    }
    
    template <typename TState, typename TAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMMDPStructure<TState, TAction>::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }


    // template <typename TState, typename TAction>
    // std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> SerializedMMDPStructure<TState,TAction>::getStateSpace() const
    // {
    //     return this->serialized_state_space_;
    // }

    template <typename TState, typename TAction>
    const std::set<TState>& SerializedMMDPStructure<TState,TAction>::getReachableSerialStates(const TState &serialized_state, const TAction& serial_action) const
    {  
        return this->reachable_state_space.at(serialized_state).at(serial_action);
    }

    template <typename TState, typename TAction>
    double SerializedMMDPStructure<TState,TAction>::getReward(const TState &s, const TAction &action) const
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

    template <typename TState, typename TAction>
    double SerializedMMDPStructure<TState,TAction>::getTransitionProbability(const TState &s,const TAction &action, const TState &s_) const
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

    template <typename TState, typename TAction>
    void SerializedMMDPStructure<TState,TAction>::setInternalState(TState new_i_state)
    {
        if(new_i_state.getCurrentAgentId() ==0)
        {
            this->mmdp_->setInternalState(new_i_state.getState());
            this->internal_state_ = new_i_state;
        }
    }

    template <typename TState, typename TAction>
    void SerializedMMDPStructure<TState,TAction>::setPlanningHorizon(number horizon)
    {
        this->mmdp_->setPlanningHorizon(horizon);
        this->planning_horizon_ = horizon;
    }

    template <typename TState, typename TAction>
    number SerializedMMDPStructure<TState,TAction>::getNumAgents() const
    {
        return this->getActionSpace()->getNumSpaces();
    }

}