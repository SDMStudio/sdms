#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    template <typename TState, typename TAction>
    SerializedMPOMDP<TState, TAction>::SerializedMPOMDP()
    {}

    template <typename TState, typename TAction>
    SerializedMPOMDP<TState, TAction>::SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_mmdp) : decpomdp_(underlying_mmdp)
    {
        this->mmdp_ = this->decpomdp_->toMMDP();

        this->setPlanningHorizon(decpomdp_->getPlanningHorizon());
        this->setDiscount(decpomdp_->getDiscount());
        this->setActionSpace(mmdp_->getActionSpace());

        this->createInitSerializedStateSpace();
        this->createInitReachableStateSpace();

        this->createInitSerialObservationSpace();
        this->createInitReachableObsStateSpace();
    }

    template <typename TState, typename TAction>
    SerializedMPOMDP<TState, TAction>::SerializedMPOMDP(std::string underlying_mmdp) : SerializedMPOMDP(std::make_shared<DiscreteDecPOMDP>(underlying_mmdp))
    {
    }

    template <typename TState, typename TAction>
    void SerializedMPOMDP<TState, TAction>::createInitSerialObservationSpace()
    {
        std::vector<std::shared_ptr<DiscreteSpace<number>>> all_observation_space;

        for(number ag_id =0; ag_id<this->getNumAgents();ag_id++)
        {
            auto obs = this->decpomdp_->getObsSpace()->getSpace(ag_id)->getAll();
            std::vector<number> observation =  obs;
            observation.push_back(obs.size());

            auto serial_ind_space = std::make_shared<DiscreteSpace<number>>(observation);

            all_observation_space.push_back(serial_ind_space);
        }
        this->serialized_observation_space_ = std::make_shared<MultiDiscreteSpace<number>>(all_observation_space);
    }

    template <typename TState, typename TAction>
    void SerializedMPOMDP<TState, TAction>::createInitReachableObsStateSpace()
    {
        for(const auto action : this->getActionSpace()->getAll())
        {
            std::unordered_map<TState,std::set<number>> map_serial_state_obs;

            for(const auto serialized_state : this->serialized_state_space_->getAll())
            {
                
                std::vector<TAction> serial_action(serialized_state.getAction());
                serial_action.push_back(action);

                std::set<number> all_obs;


                if(serialized_state.getCurrentAgentId() +1 == this->getNumAgents())
                {
                    Joint<TAction> joint_action(serial_action);
                    try
                    {
                        std::cout<<"\n next Reachable : "<<this->decpomdp_->getReacheableObservations(joint_action,serialized_state.getState());
                        all_obs = this->decpomdp_->getReacheableObservations(joint_action,serialized_state.getState());
                    }
                    catch(const std::exception& e)
                    {
                    }
                }else
                {
                    std::cout<<"\n "<<this->serialized_observation_space_->getNumJointItems();
                    all_obs.insert(this->serialized_observation_space_->getItem(this->serialized_observation_space_->getNumJointItems()));
                }
                map_serial_state_obs.emplace(serialized_state.getState(),all_obs);
            }
            this->reachable_obs_state_space.emplace(action,map_serial_state_obs);
        }
    }

    template <typename TState, typename TAction>
    const std::set<number>& SerializedMPOMDP<TState, TAction>::getReacheableObservations(const TAction& serial_action, const TState& serial_state)
    {
        return this->reachable_state_space.at(serial_action).at(serial_state);
    }

    template <typename TState, typename TAction>
    std::shared_ptr<SerializedMMDP<>> SerializedMPOMDP<TState, TAction>::toMDP()
    {
        return std::make_shared<SerializedMMDP<>>(this->decpomdp_->toMMDP());
    }
    
    template <typename TState, typename TAction>
    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMPOMDP<TState, TAction>::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    // // A partir d'ici 
    // template <typename TState, typename TAction>
    // std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP<TState,TAction>::getHiddenStateSpace() const
    // {
    //     std::vector<number> all_hidden_state;
    //     for(const auto &s : this->getStateSpaceAt(0)->getAll())
    //     {
    //         all_hidden_state.push_back(s.getState());
    //     }

    //     return std::make_shared<DiscreteSpace<number>>(all_hidden_state);
    // // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<DiscreteSpace<typename TState::state_type>> SerializedMPOMDP<TState,TAction>::getStateSpaceAt(number ag_id) const
    // {
    //     return this->getSerializedStateSpaceAt(ag_id);
    // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<MultiSpace<DiscreteSpace<typename TState::state_type>>> SerializedMPOMDP<TState,TAction>::getStateSpace() const
    // {
    //     return this->getSerializedStateSpace();
    // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<DiscreteSpace<typename TState::state_type>> SerializedMPOMDP<TState,TAction>::getSerializedStateSpaceAt(number ag_id) const
    // {
    //     return this->serialized_state_space_->getSpace(ag_id);
    // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<MultiSpace<DiscreteSpace<typename TState::state_type>>> SerializedMPOMDP<TState,TAction>::getSerializedStateSpace() const
    // {
    //     return this->serialized_state_space_;
    // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<DiscreteSpace<typename TState::state_type>> SerializedMPOMDP<TState,TAction>::getNextSerializedStateSpace(const typename TState::state_type &serialized_state) const
    // {
    //     std::vector<typename TState::state_type> all_state;
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
    //         return std::make_shared<DiscreteSpace<typename TState::state_type>>(all_state);
    //     }
    // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<DiscreteSpace<typename TState::state_type>> SerializedMPOMDP<TState,TAction>::getNextSerializedStateSpace(const typename TState::state_type &serialized_state, const number &action) const
    // {
    //     const number ag_id = serialized_state.getCurrentAgentId();

    //     if(ag_id +1 == this->getNumAgents())
    //     {
    //         return this->getSerializedStateSpaceAt(0);
    //     }
    //     else
    //     {
    //         std::vector<typename TState::state_type> all_state;

    //         std::vector<number> all_action = serialized_state.getAction();
    //         all_action.push_back(action);

    //         typename TState::state_type next_serialized_state(serialized_state.getState(),all_action) ;
            
    //         return std::make_shared<DiscreteSpace<typename TState::state_type>>(all_state);
    //     }
    // }

    // template <typename TState, typename TAction>
    // double SerializedMPOMDP<TState,TAction>::getReward(const typename TState::state_type &serialized_state, const number &action) const
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

    // template <typename TState, typename TAction>
    // double SerializedMPOMDP<TState,TAction>::getReward(const typename TState::state_type &serialized_state, const Joint<number> &action) const
    // {
    //     if( (serialized_state.getCurrentAgentId() == 0 and action.size() == this->getNumAgents()) or serialized_state.getAction()== action)
    //     {
    //         return this->decpomdp_->getReward(serialized_state.getState(),action);
    //     }else
    //     {
    //         return 0;
    //     }
    // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<Reward> SerializedMPOMDP<TState,TAction>::getReward() const
    // {
    //     return this->decpomdp_->getReward();
    // }

    template <typename TState, typename TAction>
    std::shared_ptr<MultiDiscreteSpace<number>> SerializedMPOMDP<TState,TAction>::getObsSpace() const
    {
        return this->serialized_observation_space_;
    }

    template <typename TState, typename TAction>
    std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP<TState,TAction>::getObsSpaceAt(number ag_id) const
    {
        return this->decpomdp_->getObsSpace()->getSpace(ag_id);
    }

    template <typename TState, typename TAction>
    double getObservationProbability(const TAction &action, const Joint<number> joint_obs,const TState &s)
    {
        //this->decpomdp_->getObsDynamics()->
    }

    // template <typename TState, typename TAction>
    // double SerializedMPOMDP<TState,TAction>::getObsDynamics(const number &serialized_state,const number joint_action,const number joint_obs,const number &serialized_state_next) const
    // {
    //     return this->decpomdp_->getObsDynamics()->getDynamics(serialized_state,joint_action,joint_obs,serialized_state_next);
    // }

    template <typename TState, typename TAction>
    double SerializedMPOMDP<TState,TAction>::getDynamics(const TState &serialized_state,const TAction action,const Joint<number> joint_obs,const TState &serialized_state_next) const
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
        return this->decpomdp_->getObsDynamics()->getDynamics(serialized_state.getState(),this->getActionSpace()->joint2single(Joint<number>(all_action)),this->getObsSpace()->joint2single(joint_obs),serialized_state_next.getState());
    }

    // template <typename TState, typename TAction>
    // std::shared_ptr<MultiDiscreteSpace<number>> SerializedMPOMDP<TState,TAction>::getActionSpace() const
    // {
    //     return this->decpomdp_->getActionSpace();
    // }

    // template <typename TState, typename TAction>
    // std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP<TState,TAction>::getActionSpaceAt(number ag_id) 
    // {
    //     return this->decpomdp_->getActionSpace()->getSpace(ag_id);
    // }

    // template <typename TState, typename TAction>
    // double SerializedMPOMDP<TState,TAction>::getDynamics(const typename TState::state_type &serialized_state,const number &action, const typename TState::state_type &serialized_state_next) const
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

    // template <typename TState, typename TAction>
    // number SerializedMPOMDP<TState,TAction>::getNumAgents() const
    // {
    //     return this->decpomdp_->getNumAgents();
    // }

    // template <typename TState, typename TAction>
    // void SerializedMPOMDP<TState,TAction>::setPlanningHorizon(number horizon)
    // {
    //     this->decpomdp_->setPlanningHorizon(horizon);
    //     this->planning_horizon_ = horizon;
    // }

    // template <typename TState, typename TAction>
    // std::discrete_distribution<number> SerializedMPOMDP<TState,TAction>::getStartDistrib() const
    // {
    //     return this->decpomdp_->getStartDistrib();
    // }

    // template <typename TState, typename TAction>
    // void SerializedMPOMDP<TState,TAction>::setInternalState(typename TState::state_type new_i_state)
    // {
    //     if(new_i_state.getCurrentAgentId() ==0)
    //     {
    //         this->mmdp_->setInternalState(new_i_state.getState());
    //         this->internal_state_ = new_i_state;
    //     }
    // }
}