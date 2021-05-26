#include <sdm/world/serialized_mmdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{
    
    SerializedMMDPStructure::SerializedMMDPStructure()
    {
    }

    SerializedMMDPStructure::SerializedMMDPStructure(std::shared_ptr<DiscreteMMDP> underlying_mmdp) : mmdp_(underlying_mmdp)
    {
        // Set parameter for the Serial problem
        this->setPlanningHorizon(mmdp_->getPlanningHorizon());
        this->setDiscount(mmdp_->getDiscount());
        this->setActionSpace(mmdp_->getActionSpace());
        this->setStartDistrib(mmdp_->getStartDistrib());
        
        // Creation of Serial State
        this->createInitSerializedStateSpace();

        // Creation of Reachable State
        this->createInitReachableStateSpace();

        this->setIndexAction();
    }

    
    SerializedMMDPStructure::SerializedMMDPStructure(std::string filename) : SerializedMMDPStructure(std::make_shared<DiscreteMMDP>(filename))
    {
    }

    
    void SerializedMMDPStructure::createInitSerializedStateSpace()
    {
        // Vector of all serial state
        std::vector<std::shared_ptr<DiscreteSpace<SerializedState>>> all_serialized_state;

        number n_agents = this->getNumAgents();

        std::vector<std::vector<number>> all_past_action;

        // Go over all agent
        for(int i =0; i<n_agents;i++)
        {
            // Vector all serial state
            std::vector<SerializedState> serialized_state_i;

            // All possible vector of actions
            std::vector<std::vector<number>> all_new_action;

            //Creation of all possible vector of actions
            if(i>0)
            {
                // Go over all current serial state
                for(const auto &action : all_past_action)
                {
                    // Add new action to current serial state
                    for(const auto &action_agent_i : this->mmdp_->getActionSpace()->getSpaces()[i-1]->getAll())
                    {
                        //Current action
                        std::vector<number> temp_action = action;
                        //Add new action 
                        temp_action.push_back(action_agent_i);
                        // Add new possibility in the vector
                        all_new_action.push_back(temp_action);
                    }
                }
            }else
            {
                all_new_action.push_back({});
            }

            //Go over all state 
            for(const auto &state : this->mmdp_->getStateSpace()->getAll())
            {
                // Go over all possible vector of actions
                for(const auto &action : all_new_action)
                {
                    // Add new serial state with the state of the problem and vector of action
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

    void SerializedMMDPStructure::createInitReachableStateSpace()
    {
        for(const auto serialized_state : this->serialized_state_space_->getAll())
        {
            auto x = serialized_state.getState();
            auto u = serialized_state.getAction();
            number agent_identifier = serialized_state.getCurrentAgentId();

            this->reachable_state_space.emplace(serialized_state,std::unordered_map<number,std::set<SerializedState>>());

            for(auto action : this->getActionSpace(agent_identifier)->getAll())
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
                    all_next_serial_state.insert(SerializedState(x,serial_action));
                }
                this->reachable_state_space[serialized_state].emplace(action,all_next_serial_state);
            }
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
        std::cout << "LA";

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
            return this->mmdp_->getStateDynamics()->getTransitionProbability(s.getState(),this->getJointActionSpace()->joint2single(Joint<number>(all_action)),s_.getState());
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
        return this->mmdp_->getActionSpace()->getNumSpaces();
    }

    std::shared_ptr<DiscreteSpace<number>> SerializedMMDPStructure::getActionSpace(number t) const
    {
        return this->mmdp_->getActionSpace()->getSpace(t % this->getNumAgents());
    }

    std::shared_ptr<MultiDiscreteSpace<number>> SerializedMMDPStructure::getJointActionSpace() const
    {
        return this->mmdp_->getActionSpace();
    }

    std::shared_ptr<DiscreteSpace<SerializedState>> SerializedMMDPStructure::getStateSpace(number t) const
    {
        return this->serialized_state_space_->getSpace(t % this->getNumAgents());
    }

    void SerializedMMDPStructure::setIndexAction()
    {
        for(number ag_id = 0; ag_id <this->getNumAgents(); ag_id ++)
        {
            for(auto action : this->getActionSpace(ag_id)->getAll())
            {
                this->associate_ag_id_action.push_back(std::make_pair(ag_id,action));
            }
        }
    }

    number SerializedMMDPStructure::getIndexAction(number ag_id, number action)
    {
        auto it = std::find(this->associate_ag_id_action.begin(), this->associate_ag_id_action.end(), std::make_pair(ag_id,action));

        if (it != associate_ag_id_action.end())
        {
           std::cout << "Element Found" << std::endl;
           return std::distance(associate_ag_id_action.begin(), it);;
        }
        else
        {
            std::cout << "Element Not Found" << std::endl;
            return -1;
        }
    }

}