#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/parser/parser.hpp>

namespace sdm
{

    SerializedMPOMDP::SerializedMPOMDP()
    {
    }

    SerializedMPOMDP::SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_mmdp) : decpomdp_(underlying_mmdp)
    {
        this->mmdp_ = this->decpomdp_->toMMDP();

        this->setPlanningHorizon(decpomdp_->getPlanningHorizon());
        this->setDiscount(decpomdp_->getDiscount());
        this->setActionSpace(this->mmdp_->getActionSpace());


        this->createInitSerializedStateSpace();
        this->createInitReachableStateSpace();

        this->createInitSerialObservationSpace();
        this->createInitReachableObsStateSpace();
    }

    
    SerializedMPOMDP::SerializedMPOMDP(std::string filename) : SerializedMPOMDP(std::make_shared<DiscreteDecPOMDP>(filename))
    {
    }

    
    void SerializedMPOMDP::createInitSerialObservationSpace()
    {
        std::vector<number> v;
        std::vector<std::shared_ptr<DiscreteSpace<number>>> all_observation_space;

        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            auto obs = this->decpomdp_->getObsSpace()->getSpace(ag_id)->getAll();
            std::vector<number> observation = obs;
            observation.push_back(obs.size());
            v.push_back(obs.size());

            auto serial_ind_space = std::make_shared<DiscreteSpace<number>>(observation);
            all_observation_space.push_back(serial_ind_space);
        }
        this->serialized_observation_space_ = std::make_shared<MultiDiscreteSpace<number>>(all_observation_space);
        this->empty_serial_observation = Joint<number>(v);
        std::cout<<"\n empty serial obs "<<this->empty_serial_observation;
    }

    
    void SerializedMPOMDP::createInitReachableObsStateSpace()
    {
       for (const auto serialized_state : this->serialized_state_space_->getAll())
        { 
            this->reachable_obs_state_space.emplace(serialized_state,std::unordered_map<number, std::unordered_map<SerializedState, std::set<Joint<number>>>>());
            std::vector<number> all_action(serialized_state.getAction());

            for (const auto serial_action : this->getActionSpace(serialized_state.getCurrentAgentId())->getAll())
            {
                this->reachable_obs_state_space[serialized_state].emplace(serial_action,std::unordered_map<SerializedState, std::set<Joint<number>>>());

                for (const auto next_serialized_state : this->serialized_state_space_->getAll())
                {
                    std::set<Joint<number>> all_obs;

                    if (next_serialized_state.getCurrentAgentId() == 0 && serialized_state.getCurrentAgentId() == this->getNumAgents()-1)
                    {
                        all_action.push_back(serial_action);
                        Joint<number> joint_action(all_action);
                        try
                        {
                            for (const auto obs : this->decpomdp_->getReachableObservations(serialized_state.getState(),joint_action, next_serialized_state.getState()))
                            {
                                all_obs.insert(obs);
                            }
                        }
                        catch (const std::exception &e)
                        {
                            all_obs.insert(this->serialized_observation_space_->single2joint(this->serialized_observation_space_->getNumJointItems() - 1));
                        }
                    }
                    else
                    {
                        all_obs.insert(this->serialized_observation_space_->single2joint(this->serialized_observation_space_->getNumJointItems() - 1));
                    }
                    this->reachable_obs_state_space[serialized_state][serial_action].emplace(next_serialized_state, all_obs);
                }
            }
        }
    }

    
    const std::set<Joint<number>> &SerializedMPOMDP::getReachableObservations(const SerializedState serial_state, const number serial_action, const SerializedState next_serial_state) const
    {
        return this->reachable_obs_state_space.at(serial_state).at(serial_action).at(next_serial_state);
    }

    
    std::shared_ptr<SerializedMMDP> SerializedMPOMDP::toMDP()
    {
        return std::make_shared<SerializedMMDP>(this->decpomdp_->toMMDP());
    }

    std::shared_ptr<BeliefMDP<BeliefState, number, number>> SerializedMPOMDP::toBeliefMDP()
    {
        throw sdm::exception::NotImplementedException();
    }

    std::shared_ptr<MultiDiscreteSpace<number>> SerializedMPOMDP::getObsSpace() const
    {
        return this->serialized_observation_space_;
    }

    std::shared_ptr<DiscreteSpace<number>> SerializedMPOMDP::getObsSpaceAt(number ag_id) const
    {
        return this->decpomdp_->getObsSpace()->getSpace(ag_id);
    }

    
    double SerializedMPOMDP::getObservationProbability(const SerializedState serialized_state, const number action, const Joint<number> joint_obs, const SerializedState serialized_state_next) const
    {
        std::vector<number> next_action_set = serialized_state.getAction();
        next_action_set.push_back(action);

        if (serialized_state_next.getCurrentAgentId() != 0)
        {
            return (this->empty_serial_observation == joint_obs && serialized_state.getState() == serialized_state_next.getState() && serialized_state_next.getAction() == next_action_set) ? 1 : 0;
        }
        else if( this->decpomdp_->getObsSpace()->contains(joint_obs) )
        {
            return this->decpomdp_->getObsDynamics()->getObservationProbability(serialized_state.getState(), this->getJointActionSpace()->joint2single(Joint<number>(next_action_set)), this->getObsSpace()->joint2single(joint_obs), serialized_state_next.getState());
        }
        return 0;
    }

    
    double SerializedMPOMDP::getDynamics(const SerializedState serialized_state, const number action, const Joint<number> joint_obs, const SerializedState serialized_state_next) const
    {
        std::vector<number> next_action_set = serialized_state.getAction();
        next_action_set.push_back(action);
        
        if (serialized_state_next.getCurrentAgentId() != 0)
        {
            return (this->empty_serial_observation == joint_obs && serialized_state.getState() == serialized_state_next.getState() && serialized_state_next.getAction() == next_action_set) ? 1 : 0;
        }

        else if( this->decpomdp_->getObsSpace()->contains(joint_obs) )
        {
            return this->decpomdp_->getObsDynamics()->getDynamics(serialized_state.getState(), this->getJointActionSpace()->joint2single(Joint<number>(next_action_set)), this->getObsSpace()->joint2single(joint_obs), serialized_state_next.getState());
        }
        
        return 0;
    }

}