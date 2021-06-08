/**
 * @file discrete_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file that contains the MDP class.
 * @version 1.0
 * @date 02/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <sdm/types.hpp>
#include <sdm/world/serialized_mmdp.hpp>

namespace sdm
{
    SerializedMMDP::SerializedMMDP(const std::shared_ptr<MDPInterface> &mmdp) : mmdp_(mmdp)
    {
        this->createInitSerializedStateSpace();
        this->createInitReachableStateSpace();
    }

    number SerializedMMDP::getNumAgents() const
    {
        return this->mmdp_->getNumAgents();
    }

    number SerializedMMDP::getAgentId(number t) const
    {
        return (t % this->getNumAgents());
    }

    bool SerializedMMDP::isLastAgent(number t) const
    {
        return (this->getAgentId(t) != (this->getNumAgents() - 1));
    }

    double SerializedMMDP::getDiscount(number t) const
    {
        return (((t + 1) % this->getNumAgents()) == 0) ? this->mmdp_->getDiscount(t / this->getNumAgents()) : 1.0;
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> SerializedMMDP::getStartDistribution() const
    {
        return this->mmdp_->getStartDistribution();
    }

    std::vector<std::shared_ptr<State>> SerializedMMDP::getAllStates(number t) const
    {
        return this->serialized_state_space_->getSpace(this->getAgentId(t))->getAll();
    }

    std::set<std::shared_ptr<State>> SerializedMMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->state_dynamics_->getReachableStates(state,action,t);
    }

    std::vector<std::shared_ptr<Action>> SerializedMMDP::getAllActions(number t) const
    {
        return std::static_pointer_cast<Joint<std::shared_ptr<Space<Action>>>>(this->mmdp_->getAllActions(t))->get(this->getAgentId(t));
    }

    double SerializedMMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &serial_action, number t) const
    {
        if (!this->isLastAgent(t))
        {
            return 0;
        }
        else
        {
            std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);
            auto joint_action = serialized_state->getAction();
            joint_action.push_back(serial_action);

            return this->mmdp_->getReward(serialized_state->getHiddenState(), this->getPointeurJointAction(joint_action), t);
        }
    }

    double SerializedMMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        // return this->state_dynamics_->getTransitionProbability(state,action,next_state,t);

        std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);
        std::shared_ptr<SerializedState> next_serialized_state = std::static_pointer_cast<SerializedState>(next_state);
        
        auto all_action = serialized_state->getAction();
        all_action.push_back(action);

        if (!this->isLastAgent(t))
        {
            // If the next serialized_state and the current serialized_state don't have the same hidden or it's not the same player to act, then the dynamics is impossible
            return !(((serialized_state->getCurrentAgentId() + 1) != next_serialized_state->getCurrentAgentId()) ||
                    (serialized_state->getHiddenState() != next_serialized_state->getHiddenState()) ||
                    (next_serialized_state->getAction() != all_action));
        }
        else
        {
            return this->mmdp_->getTransitionProbability(serialized_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serialized_state->getHiddenState(),t);
        }
    }

    void SerializedMMDP::createInitSerializedStateSpace()
    {
        // Vector of all serial state
        std::vector<std::shared_ptr<DiscreteSpace<std::shared_ptr<State>>>> all_serialized_state;

        std::vector<Joint<std::shared_ptr<Action>>> all_past_action;

        // Go over all agent
        for(int i =0; i<this->getNumAgents();i++)
        {
            // Vector all serial state
            std::vector<std::shared_ptr<State>> serialized_state_i;

            // All possible vector of actions
            std::vector<Joint<std::shared_ptr<Action>>> all_new_action;

            //Creation of all possible vector of actions
            if(i>0)
            {
                // Go over all current serial state
                for(const auto &action : all_past_action)
                {
                    // Add new action to current serial state
                    for(const auto &action_agent_i : this->getAllActions(i-1))
                    {
                        //Current action
                        auto temp_action = action;
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
            for(const auto &state : this->mmdp_->getAllStates(0))
            {
                // Go over all possible vector of actions
                for(const auto &action : all_new_action)
                {
                    // Add new serial state with the state of the problem and vector of action
                    auto nex_serial_state = SerializedState(state,action);

                    this->map_serialized_state_to_pointeur.emplace(nex_serial_state);
                    serialized_state_i.push_back(this->getPointeurState(nex_serial_state));
                }
            }

            this->setJointActionToPointeur(all_new_action);


            all_past_action = all_new_action;
            auto s_i = std::make_shared<DiscreteSpace<std::shared_ptr<State>>>(serialized_state_i);
            all_serialized_state.push_back(s_i);
        }
        this->serialized_state_space_= std::make_shared<MultiSpace<DiscreteSpace<std::shared_ptr<State>>>>(all_serialized_state);
    }

    void SerializedMMDP::createInitReachableStateSpace()
    {
        for(const auto state : this->serialized_state_space_->getAll())
        {
            std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);

            auto hidden_state = serialized_state->getHiddenState();
            auto action = serialized_state->getAction();
            number agent_identifier = serialized_state->getCurrentAgentId();

            auto next_action = action;

            for(auto serial_action : this->getAllActions(agent_identifier))
            {
                next_action.push_back(serial_action);

                std::set<std::shared_ptr<State>> all_next_serial_state;

                if(agent_identifier +1 == this->getNumAgents())
                {                    
                    for(const auto next_state : this->mmdp_->getReachableStates(hidden_state, this->getPointeurJointAction(next_action),agent_identifier +1 ))
                    {
                        all_next_serial_state.insert( this->getPointeurState(SerializedState(next_state,Joint<std::shared_ptr<Action>>())));
                    }
                }else
                {
                    all_next_serial_state.insert(  this->getPointeurState(SerializedState(hidden_state,next_action))) ;
                }
            }
        }
    }

    void SerializedMMDP::createInitReachableStateSpace()
    {
        auto dynamics = std::make_shared<TabularStateDynamics>();

        for(const auto state : this->serialized_state_space_->getAll())
        {
            std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);

            auto hidden_state = serialized_state->getHiddenState();
            auto action = serialized_state->getAction();
            number agent_identifier = serialized_state->getCurrentAgentId();

            auto next_action = action;

            for(auto serial_action : this->getAllActions(agent_identifier))
            {
                next_action.push_back(serial_action);

                if(agent_identifier +1 == this->getNumAgents())
                {                    
                    for(const auto next_hidden_state : this->mmdp_->getReachableStates(hidden_state, this->getPointeurJointAction(next_action),agent_identifier +1 ))
                    {
                        auto next_state = SerializedState(next_hidden_state,Joint<std::shared_ptr<Action>>());
                        dynamics->setReachablesStates(state,serial_action,this->getPointeurState(next_state));
                    }
                }else
                {
                    auto next_state = SerializedState(hidden_state,next_action);
                    dynamics->setReachablesStates(state,serial_action,this->getPointeurState(next_state));
                }
            }
        }
        this->state_dynamics_ = dynamics;
    }

    double SerializedMMDP::getMinReward(number t) const
    {
        return this->isLastAgent(t) ? this->mmdp_->getMinReward(t) : 0 ;
    }

    double SerializedMMDP::getMaxReward(number t) const
    {
        return this->isLastAgent(t) ? this->mmdp_->getMaxReward(t) : 0 ;
    }

    const std::shared_ptr<Action> SerializedMMDP::getPointeurJointAction(Joint<std::shared_ptr<Action>> &joint_action) const
    {
        return this->map_joint_action_to_pointeur.at(joint_action);
    }

    const std::shared_ptr<State> SerializedMMDP::getPointeurState(SerializedState & serialized_state) const
    {
        return this->map_serialized_state_to_pointeur.at(serialized_state);        
    }

    void SerializedMMDP::setJointActionToPointeur(std::vector<Joint<std::shared_ptr<Action>>> vecteur_of_joint_action )
    {

        // Go over all element in the vector of joint action
        for(const auto &new_joint_action : vecteur_of_joint_action )
        {
            // If it's the last agent, then find the pointeur in the mmdp associated, else create a new pointeur
            if(new_joint_action.size() + 1 == this->getNumAgents())
            {
                //Find the joint action in the mmdp associated 
                int index = std::distance(this->mmdp_->getAllActions(0).begin(), std::find(this->mmdp_->getAllActions(0).begin(),this->mmdp_->getAllActions(0).end(),new_joint_action));
                 
                this->map_joint_action_to_pointeur.emplace(new_joint_action,this->mmdp_->getAllActions(0)[index]);
            }else
            {
                this->map_joint_action_to_pointeur.emplace(new_joint_action,std::make_shared<Action>(new_joint_action));
            }
        }   
    }

} // namespace sdm