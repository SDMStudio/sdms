#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    template <typename oState, typename oAction>
    SerializedMPOMDP<oState, oAction>::SerializedMPOMDP()
    {}

    template <typename oState, typename oAction>
    SerializedMPOMDP<oState, oAction>::SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_mmdp) : decpomdp_(underlying_mmdp)
    {
        this->setPlanningHorizon(decpomdp_->getPlanningHorizon());
        this->setDiscount(decpomdp_->getDiscount());


        //Creation of all Serialized_state
        std::vector<std::shared_ptr<DiscreteSpace<typename oState::state_type>>> all_serialized_state;

        number n_agents = this->getNumAgents();
        std::vector<std::vector<number>> all_past_action;

        for(int i =0; i<n_agents;i++)
        {
            std::vector<typename oState::state_type> serialized_state_i;
            std::vector<std::vector<number>> all_new_action;

            if(i>0)
            {
                for(const auto &action : all_past_action)
                {
                    for(const auto &action_agent_i : this->getActionSpaceAt(i-1)->getAll())
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
            for(const auto &state : this->decpomdp_->getStateSpace()->getAll())
            {
                for(const auto &action : all_new_action)
                {
                    serialized_state_i.push_back(typename oState::state_type(state,action));
                }
            }
            all_past_action = all_new_action;
            auto s_i = std::make_shared<DiscreteSpace<typename oState::state_type>>(serialized_state_i);
            all_serialized_state.push_back(s_i);
        }

        this->serialized_state_space_= std::make_shared<MultiSpace<DiscreteSpace<typename oState::state_type>>>(all_serialized_state);

        this->serialized_observation_space_= this->decpomdp_->getObsSpace();
    }

    template <typename oState, typename oAction>
    SerializedMPOMDP<oState, oAction>::SerializedMPOMDP(std::string underlying_mmdp) : SerializedMPOMDP(std::make_shared<DiscreteDecPOMDP>(underlying_mmdp))
    {
    }

    template <typename oState, typename oAction>
    std::shared_ptr<SerializedMMDP<>> SerializedMPOMDP<oState, oAction>::toMDP()
    {
        return std::make_shared<SerializedMMDP<>>(this->decpomdp_->toMMDP());
    }
    
    template <typename oState, typename oAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMPOMDP<oState, oAction>::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    // // A partir d'ici 
    // template <typename oState, typename oAction>
    // std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP<oState,oAction>::getHiddenStateSpace() const
    // {
    //     std::vector<number> all_hidden_state;
    //     for(const auto &s : this->getStateSpaceAt(0)->getAll())
    //     {
    //         all_hidden_state.push_back(s.getState());
    //     }

    //     return std::make_shared<DiscreteSpace<number>>(all_hidden_state);
    // // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<DiscreteSpace<typename oState::state_type>> SerializedMPOMDP<oState,oAction>::getStateSpaceAt(number ag_id) const
    // {
    //     return this->getSerializedStateSpaceAt(ag_id);
    // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<MultiSpace<DiscreteSpace<typename oState::state_type>>> SerializedMPOMDP<oState,oAction>::getStateSpace() const
    // {
    //     return this->getSerializedStateSpace();
    // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<DiscreteSpace<typename oState::state_type>> SerializedMPOMDP<oState,oAction>::getSerializedStateSpaceAt(number ag_id) const
    // {
    //     return this->serialized_state_space_->getSpace(ag_id);
    // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<MultiSpace<DiscreteSpace<typename oState::state_type>>> SerializedMPOMDP<oState,oAction>::getSerializedStateSpace() const
    // {
    //     return this->serialized_state_space_;
    // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<DiscreteSpace<typename oState::state_type>> SerializedMPOMDP<oState,oAction>::getNextSerializedStateSpace(const typename oState::state_type &serialized_state) const
    // {
    //     std::vector<typename oState::state_type> all_state;
    //     const number ag_id = serialized_state.getCurrentAgentId();

    //     if(ag_id +1 == this->getNumAgents())
    //     {
    //         return this->getSerializedStateSpaceAt(0);
    //     }
    //     else
    //     {
    //         for(const auto state : this->getSerializedStateSpaceAt(ag_id +1)->getAll())
    //         {
    //             if(state.getState() == serialized_state.getState())
    //             {
    //                 std::vector<number> action = state.getAction();
    //                 action.pop_back();
    //                 if(action == serialized_state.getAction())
    //                 {
    //                     all_state.push_back(state);
    //                 }
    //             }
    //         }
    //         return std::make_shared<DiscreteSpace<typename oState::state_type>>(all_state);
    //     }
    // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<DiscreteSpace<typename oState::state_type>> SerializedMPOMDP<oState,oAction>::getNextSerializedStateSpace(const typename oState::state_type &serialized_state, const number &action) const
    // {
    //     const number ag_id = serialized_state.getCurrentAgentId();

    //     if(ag_id +1 == this->getNumAgents())
    //     {
    //         return this->getSerializedStateSpaceAt(0);
    //     }
    //     else
    //     {
    //         std::vector<typename oState::state_type> all_state;

    //         std::vector<number> all_action = serialized_state.getAction();
    //         all_action.push_back(action);

    //         typename oState::state_type next_serialized_state(serialized_state.getState(),all_action) ;
            
    //         return std::make_shared<DiscreteSpace<typename oState::state_type>>(all_state);
    //     }
    // }

    // template <typename oState, typename oAction>
    // double SerializedMPOMDP<oState,oAction>::getReward(const typename oState::state_type &serialized_state, const number &action) const
    // {
    //     if(serialized_state.getCurrentAgentId() +1 != this->getNumAgents())
    //     {
    //         return 0;
    //     }else
    //     {
    //         std::vector<number> all_action = serialized_state.getAction();
    //         all_action.push_back(action);
            
    //         return this->decpomdp_->getReward(serialized_state.getState(),Joint<number>(all_action));
    //     }
    // }

    // template <typename oState, typename oAction>
    // double SerializedMPOMDP<oState,oAction>::getReward(const typename oState::state_type &serialized_state, const Joint<number> &action) const
    // {
    //     if( (serialized_state.getCurrentAgentId() == 0 and action.size() == this->getNumAgents()) or serialized_state.getAction()== action)
    //     {
    //         return this->decpomdp_->getReward(serialized_state.getState(),action);
    //     }else
    //     {
    //         return 0;
    //     }
    // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<Reward> SerializedMPOMDP<oState,oAction>::getReward() const
    // {
    //     return this->decpomdp_->getReward();
    // }

    template <typename oState, typename oAction>
    std::shared_ptr<MultiDiscreteSpace<number>> SerializedMPOMDP<oState,oAction>::getObsSpace() const
    {
        return this->serialized_observation_space_;
    }

    template <typename oState, typename oAction>
    std::vector<Joint<number>> SerializedMPOMDP<oState,oAction>::getObsSpaceAt(const typename oState::state_type &serialized_state) const
    {
        if(serialized_state.getCurrentAgentId() != this->getNumAgents() -1 )
        {
            Joint<number> empty(std::vector<number> {}); 

            std::vector<Joint<number>> vector_empty;
            vector_empty.push_back(empty);

            return vector_empty;
        }
        return this->serialized_observation_space_->getAll();
    }

    template <typename oState, typename oAction>
    std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP<oState,oAction>::getObsSpaceAt(number ag_id) const
    {
        return this->decpomdp_->getObsSpace()->getSpace(ag_id);
    }

    template <typename oState, typename oAction>
    double SerializedMPOMDP<oState,oAction>::getObsDynamics(const number &serialized_state,const number joint_action,const number joint_obs,const number &serialized_state_next) const
    {
        return this->decpomdp_->getObsDynamics()->getDynamics(serialized_state,joint_action,joint_obs,serialized_state_next);
    }

    template <typename oState, typename oAction>
    double SerializedMPOMDP<oState,oAction>::getObsDynamics(const typename oState::state_type &serialized_state,const number action,const Joint<number> joint_obs,const typename oState::state_type &serialized_state_next) const
    {
        std::vector<number> all_action = serialized_state.getAction();
        all_action.push_back(action);
        if(serialized_state.getCurrentAgentId() != this->getNumAgents() -1 )
        {
            if(serialized_state.getState()== serialized_state_next.getState() and serialized_state_next.getAction()==all_action)
            {
                return 1;
            }
            return 0;
        }
        // Autre vérification, que s'.getCurrentAgentId = 0
        return this->decpomdp_->getObsDynamics()->getDynamics(serialized_state.getState(),this->getActionSpace()->joint2single(Joint<number>(all_action)),this->getObsSpace()->joint2single(joint_obs),serialized_state_next.getState());
    }

    // template <typename oState, typename oAction>
    // std::shared_ptr<MultiDiscreteSpace<number>> SerializedMPOMDP<oState,oAction>::getActionSpace() const
    // {
    //     return this->decpomdp_->getActionSpace();
    // }

    // template <typename oState, typename oAction>
    // std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP<oState,oAction>::getActionSpaceAt(number ag_id) 
    // {
    //     return this->decpomdp_->getActionSpace()->getSpace(ag_id);
    // }

    // template <typename oState, typename oAction>
    // double SerializedMPOMDP<oState,oAction>::getDynamics(const typename oState::state_type &serialized_state,const number &action, const typename oState::state_type &serialized_state_next) const
    // {
    //     if(serialized_state.getCurrentAgentId() +1 != this->getNumAgents())
    //     {
    //         if(serialized_state.getCurrentAgentId() +1 != serialized_state_next.getCurrentAgentId() or serialized_state.getState()!= serialized_state_next.getState())
    //         {
    //             return 0;
    //         }else
    //         {
    //             return 1; 
    //         }
    //     }else
    //     {
    //         std::vector<number> all_action = serialized_state.getAction();
    //         all_action.push_back(action);
    //         return this->decpomdp_->getStateDynamics()->getTransitionProbability(serialized_state.getState(),this->getActionSpace()->joint2single(Joint<number>(all_action)),serialized_state_next.getState());
    //     }
    // }

    // template <typename oState, typename oAction>
    // number SerializedMPOMDP<oState,oAction>::getNumAgents() const
    // {
    //     return this->decpomdp_->getNumAgents();
    // }

    // template <typename oState, typename oAction>
    // void SerializedMPOMDP<oState,oAction>::setPlanningHorizon(number horizon)
    // {
    //     this->decpomdp_->setPlanningHorizon(horizon);
    //     this->planning_horizon_ = horizon;
    // }

    // template <typename oState, typename oAction>
    // std::discrete_distribution<number> SerializedMPOMDP<oState,oAction>::getStartDistrib() const
    // {
    //     return this->decpomdp_->getStartDistrib();
    // }

    // template <typename oState, typename oAction>
    // void SerializedMPOMDP<oState,oAction>::setInternalState(typename oState::state_type new_i_state)
    // {
    //     if(new_i_state.getCurrentAgentId() ==0)
    //     {
    //         this->mmdp_->setInternalState(new_i_state.getState());
    //         this->internal_state_ = new_i_state;
    //     }
    // }
}