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
    }

    std::set<std::shared_ptr<State>> SerializedMMDP::getReachableStates(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const
    {
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

            return this->mmdp_->getReward(serialized_state->getHiddenState(), joint_action, t);
        }
    }

    double SerializedMMDP::getTransitionProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);
        std::shared_ptr<SerializedState> next_serialized_state = std::static_pointer_cast<SerializedState>(next_state);
        
        Joint<std::shared_ptr<Action>> all_action = serialized_state->getAction();
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
            return this->mmdp_->getTransitionProbability(serialized_state->getHiddenState(), all_action, next_serialized_state->getHiddenState());
        }
    }

    void SerializedMMDP::createInitSerializedStateSpace()
    {
        // Vector of all serial state
        std::vector<std::shared_ptr<DiscreteSpace<std::shared_ptr<State>>>> all_serialized_state;

        number n_agents = this->getNumAgents();

        std::vector<Joint<std::shared_ptr<Action>>> all_past_action;

        // Go over all agent
        for(int i =0; i<n_agents;i++)
        {
            // Vector all serial state
            std::vector<std::shared_ptr<SerializedState>> serialized_state_i;

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
                    serialized_state_i.push_back(std::make_shared<SerializedState>(state,action));
                }
            }
            all_past_action = all_new_action;
            auto s_i = std::make_shared<DiscreteSpace<std::shared_ptr<State>>>(serialized_state_i);
            all_serialized_state.push_back(s_i);
        }
        this->serialized_state_space_= std::make_shared<MultiSpace<DiscreteSpace<std::shared_ptr<SerializedState>>>>(all_serialized_state);
    }

    void SerializedMMDP::createInitReachableStateSpace()
    {
        for(const auto serialized_state : this->serialized_state_space_->getAll())
        {
            auto hidden_state = serialized_state->getHiddenState();
            auto action = serialized_state->getAction();
            number agent_identifier = serialized_state->getCurrentAgentId();

            auto next_action = action;

            this->reachable_state_space.emplace(serialized_state,std::unordered_map<number,std::set<SerializedState>>());

            for(auto serial_action : this->getAllActions(agent_identifier))
            {
                next_action.push_back(serial_action);

                std::set<SerializedState> all_next_serial_state;

                if(agent_identifier +1 == this->getNumAgents())
                {                    
                    for(const auto next_state : this->mmdp_->getReachableStates(hidden_state, next_action))
                    {
                        all_next_serial_state.insert(std::make_shared<SerializedState>(next_state,Joint<std::shared_ptr<Action>>()));
                    }
                }else
                {
                    all_next_serial_state.insert(std::shared_ptr<SerializedState>(hidden_state,next_action));
                }
                this->reachable_state_space[serialized_state].emplace(action,all_next_serial_state);
            }
        }
    }

    double SerializedMMDP::getMinReward(number t) const
    {
        return this->isLastAgent(t) ? this->mmdp_->getMinReward(t) : 0 ;
    }

    double SerializedMMDP::getMaxReward(number t) const
    {
        this->isLastAgent(t) ? this->mmdp_->getMaxReward(t) : 0 ;
    }

} // namespace sdm