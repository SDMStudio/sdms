#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
<<<<<<< HEAD
    
=======
>>>>>>> 01e62d6b25a55e3932a4bd4e5ad118ed3b293995
    SerializedMMDPStructure::SerializedMMDPStructure()
    {
    }

<<<<<<< HEAD
    
    SerializedMMDPStructure::SerializedMMDPStructure(std::shared_ptr<DiscreteMMDP> mmdp) : mmdp_(mmdp)
=======
    SerializedMMDPStructure::SerializedMMDPStructure(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
>>>>>>> 01e62d6b25a55e3932a4bd4e5ad118ed3b293995
    {
        this->setPlanningHorizon(mmdp_->getPlanningHorizon());
        this->setDiscount(mmdp_->getDiscount());
        this->setActionSpace(mmdp_->getActionSpace());
        
        this->createInitSerializedStateSpace();
        this->createInitReachableStateSpace();

        this->setStartDistrib(mmdp_->getStartDistrib());
    }

<<<<<<< HEAD
    
    SerializedMMDPStructure::SerializedMMDPStructure(std::string filename) : SerializedMMDPStructure(std::make_shared<DiscreteMMDP>(filename))
    {
    }

    
=======
    SerializedMMDPStructure::SerializedMMDPStructure(std::string underlying_mmdp) : SerializedMMDPStructure(std::make_shared<DiscreteMMDP>(underlying_mmdp))
    {
    }

>>>>>>> 01e62d6b25a55e3932a4bd4e5ad118ed3b293995
    void SerializedMMDPStructure::createInitSerializedStateSpace()
    {
        std::vector<std::shared_ptr<DiscreteSpace<SerializedState>>> all_serialized_state;

        number n_agents = this->getNumAgents();

        std::vector<std::vector<number>> all_past_action;

        for(int i =0; i<n_agents;i++)
        {
            std::vector<SerializedState> serialized_state_i;
            std::vector<std::vector<number>> all_new_action;

            if(i>0)
            {
                for(const auto &action : all_past_action)
                {
                    for(const auto &action_agent_i : this->mmdp_->getActionSpace()->getSpaces()[i-1]->getAll())
                    {
                        std::vector<number> temp_action = action;
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
                    serialized_state_i.push_back(SerializedState(state,action));
                }
            }
            all_past_action = all_new_action;
            auto s_i = std::make_shared<DiscreteSpace<SerializedState>>(serialized_state_i);
            all_serialized_state.push_back(s_i);
        }
        this->serialized_state_space_= std::make_shared<MultiSpace<DiscreteSpace<SerializedState>>>(all_serialized_state);
        this->setStateSpace(this->serialized_state_space_);
    }

<<<<<<< HEAD
    
=======
>>>>>>> 01e62d6b25a55e3932a4bd4e5ad118ed3b293995
    void SerializedMMDPStructure::createInitReachableStateSpace()
    {
        for(const auto serialized_state : this->serialized_state_space_->getAll())
        {
            auto x = serialized_state.getState();
            auto u = serialized_state.getAction();
            number agent_identifier = serialized_state.getCurrentAgentId();

            std::unordered_map<number,std::set<SerializedState>> map_action_next_serial_state;

            for(auto action : this->getActionSpace()->getSpace(agent_identifier)->getAll())
            {
                std::vector<number> serial_action(u);
                serial_action.push_back(action);

                std::set<SerializedState> all_next_serial_state;

                if(agent_identifier +1 == this->getNumAgents())
                {
                    Joint<number> joint_action(serial_action);
                    try
                    {
                        for(const auto next_state : this->mmdp_->getReachableStates(x, joint_action))
                        {
                            all_next_serial_state.insert(SerializedState(next_state,std::vector<number>()));
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

    
    double SerializedMMDPStructure::getDiscount(number t) const
    {
        return (t % this->getNumAgents() == this->getNumAgents() - 1) ? this->mmdp_->getDiscount() : 1.0;
    }

    
    std::shared_ptr<SerializedMMDPStructure> SerializedMMDPStructure::getptr()
    {
        return SerializedMMDPStructure::shared_from_this();
    }

    
    std::shared_ptr<SerializedMMDPStructure> SerializedMMDPStructure::toMDP()
    {
        return this->getptr();
    }
    
    
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMMDPStructure::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    
    const std::set<SerializedState>& SerializedMMDPStructure::getReachableSerialStates(const SerializedState &serialized_state, const number& serial_action) const
    {  
        return this->reachable_state_space.at(serialized_state).at(serial_action);
    }

    
    double SerializedMMDPStructure::getReward(const SerializedState &serialized_state, const number &serial_action) const
    {
        if(serialized_state.getCurrentAgentId() +1 != this->getNumAgents())
        {
            return 0;
        }else
        {
            std::vector<number> all_action = serialized_state.getAction();
            all_action.push_back(serial_action);

            
            return this->mmdp_->getReward(serialized_state.getState(),Joint<number>(all_action));
        }
    }

    
    double SerializedMMDPStructure::getReward(const SerializedState &serialized_state, const Joint<number> &joint_action) const
    {
        if(serialized_state.getAction()!= joint_action)
        {
            return 0;
        }else
        {
            return this->mmdp_->getReward(serialized_state.getState(),joint_action);
        }
    }


    
    std::shared_ptr<Reward> SerializedMMDPStructure::getReward() const
    {
        return this->mmdp_->getReward();
    }

    
    double SerializedMMDPStructure::getTransitionProbability(const SerializedState &s,const number &action, const SerializedState &s_) const
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

    
    void SerializedMMDPStructure::setInternalState(SerializedState new_i_state)
    {
        if(new_i_state.getCurrentAgentId() ==0)
        {
            this->mmdp_->setInternalState(new_i_state.getState());
            this->internal_state_ = new_i_state;
        }
    }

    
    void SerializedMMDPStructure::setPlanningHorizon(number horizon)
    {
        this->mmdp_->setPlanningHorizon(horizon);
        this->planning_horizon_ = horizon;
    }

    
    number SerializedMMDPStructure::getNumAgents() const
    {
        return this->getActionSpace()->getNumSpaces();
    }
}