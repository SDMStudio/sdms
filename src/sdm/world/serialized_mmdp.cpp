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
#include <sdm/core/state/interface/serial_interface.hpp>

namespace sdm
{
    SerializedMMDP::SerializedMMDP(const std::shared_ptr<MMDPInterface> &mmdp) : mmdp_(mmdp)
    {
        this->createInitSerializedStateSpace();
        this->createInitReachableStateSpace();
    }

    SerializedMMDP::~SerializedMMDP() {}


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
        return (this->getAgentId(t) == (this->getNumAgents() - 1));
    }

    double SerializedMMDP::getDiscount(number t) const
    {
        return (((t + 1) % this->getNumAgents()) == 0) ? this->mmdp_->getDiscount(t / this->getNumAgents()) : 1.0;
    }

    number SerializedMMDP::getHorizon() const
    {
        return this->mmdp_->getHorizon();
    }

    std::shared_ptr<Distribution<std::shared_ptr<State>>> SerializedMMDP::getStartDistribution() const
    {
        return this->mmdp_->getStartDistribution();
    }

    std::shared_ptr<Space> SerializedMMDP::getStateSpace(number t) const
    {
        return  this->serialized_state_space_.at(this->getAgentId(t));
    }

    std::set<std::shared_ptr<State>> SerializedMMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
        return this->state_dynamics_->getReachableStates(state,action,t);
    }

    std::shared_ptr<Space> SerializedMMDP::getActionSpace(number t) const
    {
        return this->mmdp_->getActionSpace(this->getAgentId(t),t);
    }

    std::shared_ptr<Space> SerializedMMDP::getActionSpace(number agent_id, number t) const
    {
        return this->getActionSpace(t);
    }

    double SerializedMMDP::getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &serial_action, number t) const
    {
        if (!this->isLastAgent(t))
        {
            return 0;
        }
        else
        {            
            std::shared_ptr<SerialInterface> serialized_state = std::dynamic_pointer_cast<SerialInterface>(state);
            auto joint_action = serialized_state->getAction();
            joint_action.push_back(serial_action);

            return this->mmdp_->getReward(serialized_state->getHiddenState(), this->getPointeurJointAction(joint_action), t);
        }
    }

    double SerializedMMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        std::shared_ptr<SerialInterface> serialized_state = std::dynamic_pointer_cast<SerialInterface>(state);
        std::shared_ptr<SerialInterface> next_serialized_state = std::dynamic_pointer_cast<SerialInterface>(next_state);
        
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
            return this->mmdp_->getTransitionProbability(serialized_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serialized_state->getHiddenState(), t);
        }
    }

    void SerializedMMDP::createInitSerializedStateSpace()
    {
        // Vector of all serial state
        std::vector<std::shared_ptr<DiscreteSpace>> all_serialized_state;

        std::vector<Joint<std::shared_ptr<Action>>> all_past_action;

        // Go over all agent
        for(int i =0; i<this->getNumAgents();i++)
        {
            // Vector all serial state
            std::vector<std::shared_ptr<Item>> serialized_state_i;

            // All possible vector of actions
            std::vector<Joint<std::shared_ptr<Action>>> all_new_action;

            //Creation of all possible vector of actions

            if(i>0)
            {
                // Go over all current serial state
                for(const auto &action : all_past_action)
                {
                    // Add new action to current serial state
                    for(const auto &action_agent_i : *this->getActionSpace(i-1))
                    {
                        //Current action
                        auto temp_action = action;
                        //Add new action 
                        temp_action.push_back(action_agent_i->toAction());
                        // Add new possibility in the vector
                        all_new_action.push_back(temp_action);
                    }
                }
            }else
            {
                all_new_action.push_back({});
            }

            //Go over all state 

            for(const auto &state : *this->mmdp_->getStateSpace(0))
            {
                // Go over all possible vector of actions
                for(const auto &action : all_new_action)
                {
                    // Add new serial state with the state of the problem and vector of action
                    auto next_serial_state = std::make_shared<SerializedState>(state->toState(),action);
                    std::shared_ptr<State> next_serial_state_str = std::shared_ptr<SerialInterface>(next_serial_state);

                    // map_serial_state_to_pointeur.emplace(next_serial_state,next_serial_state_str);
                    this->map_serialized_state_to_pointeur[*next_serial_state] = next_serial_state_str;

                    serialized_state_i.push_back(next_serial_state_str);
                }
            }

            this->setJointActionToPointeur(all_new_action);


            all_past_action = all_new_action;
            auto s_i = std::make_shared<DiscreteSpace>(serialized_state_i);
            all_serialized_state.push_back(s_i);
        }
        this->serialized_state_space_= Joint<std::shared_ptr<DiscreteSpace>>(all_serialized_state);

        std::vector<Joint<std::shared_ptr<Action>>> vector_joint_action;
        for(const auto &element : *this->mmdp_->getActionSpace(0))
        {
            auto joint_action = std::static_pointer_cast<Joint<std::shared_ptr<Action>>>(element);
            std::shared_ptr<Action> jaction = joint_action;
            this->map_joint_action_to_pointeur[*joint_action] =jaction;
        }

    }

    void SerializedMMDP::createInitReachableStateSpace()
    {
        auto dynamics = std::make_shared<TabularStateDynamics>();

        for(number agent_id = 0; agent_id<this->getNumAgents();agent_id ++)
        {
            for(const auto &state : *this->getStateSpace(agent_id))
            {
                auto serialized_state = state->toState()->toSerial();

                auto hidden_state = serialized_state->getHiddenState();
                auto action = serialized_state->getAction();


                for(auto action_tmp : *this->getActionSpace(agent_id))
                {
                    auto serial_action = action_tmp->toAction();

                    auto next_action = action;

                    next_action.push_back(serial_action);

                    if(agent_id +1 == this->getNumAgents())
                    {                
                        for(const auto next_hidden_state : this->mmdp_->getReachableStates(hidden_state, this->getPointeurJointAction(next_action),agent_id +1 ))
                        {
                            auto next_state = SerializedState(next_hidden_state,Joint<std::shared_ptr<Action>>());
                            dynamics->setReachablesStates(serialized_state,serial_action,this->getPointeurState(next_state));
                        }
                    }else
                    {
                        auto next_state = SerializedState(hidden_state,next_action);
                        dynamics->setReachablesStates(serialized_state,serial_action,this->getPointeurState(next_state));
                    }
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
        for(auto &new_joint_action : vecteur_of_joint_action )
        {
            std::shared_ptr<Action> jaction = std::make_shared<Joint<std::shared_ptr<Action>>>(new_joint_action);
            this->map_joint_action_to_pointeur[new_joint_action] =jaction;
        }   
    }

} // namespace sdm