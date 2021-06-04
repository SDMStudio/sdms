#include <sdm/world/serialized_mpomdp.hpp>

namespace sdm
{   
    SerializedMPOMDP::SerializedMPOMDP(std::shared_ptr<POMDPInterface> mpomdp) : SerializedMMDP(mpomdp)
    {}

    std::vector<std::shared_ptr<Observation>> SerializedMPOMDP::getAllObservations(number t) const
    {
        return this->serialized_observation_space_->getSpace(this->getAgentId(t))->getAll();
    }

    std::set<std::shared_ptr<Observation>> SerializedMPOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action,const std::shared_ptr<State> &next_state, number) const
    {
        return this->reachable_obs_state_space.at(state).at(action).at(next_state);
    }

    double SerializedMPOMDP::getObsProbability(const std::shared_ptr<State> &state,const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);
        std::shared_ptr<SerializedState> next_serialized_state = std::static_pointer_cast<SerializedState>(next_state);

        auto all_action(serialized_state->getAction());
        all_action.push_back(action);

        auto find = std::find(this->getAllObservations(serialized_state->getCurrentAgentId()).begin(), this->getAllObservations(serialized_state->getCurrentAgentId()).end(), observation);

        if(find != this->getAllObservations(serialized_state->getCurrentAgentId()).end() && next_serialized_state->getCurrentAgentId() == 0 && serialized_state->getCurrentAgentId() == this->getNumAgents() -1 && this->getAllObservations(serialized_state->getCurrentAgentId())[std::distance(this->getAllObservations(serialized_state->getCurrentAgentId()).begin(),find)] )
        {
            // The probability is the same of the decpomdp, if the condition are verified
            return std::static_pointer_cast<POMDPInterface>(this->mmdp_)->getObsProbability(serialized_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serialized_state->getHiddenState(),observation,t);
        }else
        {
            // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
            // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
            return (this->empty_serial_observation == observation && serialized_state->getHiddenState() == next_serialized_state->getHiddenState() && next_serialized_state->getAction() == all_action) ? 1 : 0;
        }
    }

    double SerializedMPOMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);
        std::shared_ptr<SerializedState> next_serialized_state = std::static_pointer_cast<SerializedState>(next_state);

        auto all_action(serialized_state->getAction());
        all_action.push_back(action);

        auto find = std::find(this->getAllObservations(serialized_state->getCurrentAgentId()).begin(), this->getAllObservations(serialized_state->getCurrentAgentId()).end(), observation);

        if(find != this->getAllObservations(serialized_state->getCurrentAgentId()).end() && next_serialized_state->getCurrentAgentId() == 0 && serialized_state->getCurrentAgentId() == this->getNumAgents() -1 && this->getAllObservations(serialized_state->getCurrentAgentId())[std::distance(this->getAllObservations(serialized_state->getCurrentAgentId()).begin(),find)] )
        {
            // The probability is the same of the decpomdp, if the condition are verified
            return std::static_pointer_cast<POMDPInterface>(this->mmdp_)->getDynamics(serialized_state->getHiddenState(), this->getPointeurJointAction(all_action),next_serialized_state->getHiddenState(),observation,t);
        }else
        {
            // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
            // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
            return (this->empty_serial_observation == observation && serialized_state->getHiddenState() == next_serialized_state->getHiddenState() && next_serialized_state->getAction() == all_action) ? 1 : 0;
        }
    }

    void SerializedMPOMDP::setupObservationSpace(std::shared_ptr<POMDPInterface> mpomdp)
    {
        // Set default joint observation : "No Observation"
        Joint<std::shared_ptr<Observation>> empty_observation;

        // Create the Default Joint Observation
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            empty_observation.push_back(std::make_shared<Observation>(sdm::DEFAULT_OBSERVATION));
        }

        // Add the information in the variable empty_serial_observation
        this->empty_serial_observation = std::make_shared<Observation>(empty_observation);

        // Create the Serial Observation Space
        std::vector<std::shared_ptr<DiscreteSpace<std::shared_ptr<Observation>>>> all_observation_space;
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            //If it's the last agent, then the serial observation space correspond to the mpomdp observation space, else it's the empty observation
            if (this->isLastAgent(ag_id))
            {
                all_observation_space.push_back(std::make_shared<DiscreteSpace<std::shared_ptr<Observation>>>(mpomdp->getAllObservations(0)));
            }
            else
            {
                all_observation_space.push_back(std::make_shared<DiscreteSpace<std::shared_ptr<Observation>>>(this->empty_serial_observation));
            }
        }
        this->serialized_observation_space_ = std::make_shared<MultiSpace<DiscreteSpace<std::shared_ptr<Observation>>>>(all_observation_space);
    }

    void SerializedMPOMDP::setReachableObservationSpace(std::shared_ptr<POMDPInterface> mpomdp)
    {
        // Creation of all reachable Observation Space

        // Go over serial states
        for (const auto state : this->serialized_state_space_->getAll())
        {
            std::shared_ptr<SerializedState> serialized_state = std::static_pointer_cast<SerializedState>(state);

            this->reachable_obs_state_space.emplace(serialized_state, std::unordered_map<std::shared_ptr<Action>, std::unordered_map<std::shared_ptr<State>, std::set<std::shared_ptr<Observation>>>>());

            // Go over serial action
            for (const auto serial_action : this->getAllActions(serialized_state->getCurrentAgentId())->getAll())
            {
                this->reachable_obs_state_space[serialized_state].emplace(serial_action, std::unordered_map<std::shared_ptr<State>, std::set<std::shared_ptr<Observation>>>());

                // Go over joint_obs
                for (const auto next_state : this->getAllStates(serialized_state->getCurrentAgentId()+1) )
                {
                    std::shared_ptr<SerializedState> next_serialized_state = std::static_pointer_cast<SerializedState>(next_state);

                    //Update action
                    auto joint_action(serialized_state->getAction());
                    joint_action.push_back(serial_action);

                    std::set<std::shared_ptr<Observation>> all_obs;

                    // Insert the next reachable Observation State
                    if (next_serialized_state->getCurrentAgentId() == 0)
                    {
                        for (const auto obs : mpomdp->getReachableObservations(serialized_state->getHiddenState(), this->getPointeurJointAction(joint_action), next_serialized_state->getHiddenState()),t)
                        {
                            all_obs.insert(obs);
                        }
                    }
                    else
                    {
                        all_obs.insert(this->empty_serial_observation);
                    }
                    this->reachable_obs_state_space[serialized_state][serial_action].emplace(next_serialized_state, all_obs);
                }
            }
        }
    }


}