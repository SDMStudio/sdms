#include <sdm/world/serialized_mpomdp.hpp>
#include <sdm/core/observation/default_observation.hpp>

namespace sdm
{
    SerializedMPOMDP::SerializedMPOMDP(std::shared_ptr<MPOMDPInterface> mpomdp) : SerializedMMDP(mpomdp)
    {
        setupObservationSpace(mpomdp);
        setReachableObservationSpace(mpomdp);
    }

    std::shared_ptr<Space> SerializedMPOMDP::getObservationSpace(number t) const
    {
        return this->serialized_observation_space_.at(this->getAgentId(t));
    }

    std::shared_ptr<Space> SerializedMPOMDP::getObservationSpace(number, number t) const
    {
        return this->getObservationSpace(t);
    }

    std::set<std::shared_ptr<Observation>> SerializedMPOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->obs_dynamics_->getReachableObservations(state, action, next_state, t);
    }

    double SerializedMPOMDP::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        std::shared_ptr<SerializedState> serialized_state = std::dynamic_pointer_cast<SerializedState>(state);
        std::shared_ptr<SerializedState> next_serialized_state = std::dynamic_pointer_cast<SerializedState>(next_state);

        auto all_action(serialized_state->getAction());
        all_action.push_back(action);

        std::shared_ptr<Item> item = observation;
        auto discrete_space = std::static_pointer_cast<DiscreteSpace>(this->getObservationSpace(serialized_state->getCurrentAgentId()));

        auto index = discrete_space->find(item);

        if (index != -1 && next_serialized_state->getCurrentAgentId() == 0 && serialized_state->getCurrentAgentId() == this->getNumAgents() - 1 && discrete_space->getItem(index))
        {
            // The probability is the same of the decpomdp, if the condition are verified
            return std::dynamic_pointer_cast<MPOMDPInterface>(this->mmdp_)->getObservationProbability(serialized_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serialized_state->getHiddenState(), observation, t);
        }
        else
        {
            // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
            // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
            return (this->empty_serial_observation == observation && serialized_state->getHiddenState() == next_serialized_state->getHiddenState() && next_serialized_state->getAction() == all_action) ? 1 : 0;
        }
    }

    double SerializedMPOMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        std::shared_ptr<SerializedState> serialized_state = std::dynamic_pointer_cast<SerializedState>(state);
        std::shared_ptr<SerializedState> next_serialized_state = std::dynamic_pointer_cast<SerializedState>(next_state);

        auto all_action(serialized_state->getAction());
        all_action.push_back(action);

        std::shared_ptr<Item> item = observation;
        auto discrete_space = std::static_pointer_cast<DiscreteSpace>(this->getObservationSpace(serialized_state->getCurrentAgentId()));

        auto index = discrete_space->find(item);

        if (index != -1 && next_serialized_state->getCurrentAgentId() == 0 && serialized_state->getCurrentAgentId() == this->getNumAgents() - 1 && discrete_space->getItem(index))
        {
            // The probability is the same of the decpomdp, if the condition are verified
            return std::dynamic_pointer_cast<MPOMDPInterface>(this->mmdp_)->getDynamics(serialized_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serialized_state->getHiddenState(), observation, t);
        }
        else
        {
            // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
            // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
            return (this->empty_serial_observation == observation && serialized_state->getHiddenState() == next_serialized_state->getHiddenState() && next_serialized_state->getAction() == all_action) ? 1 : 0;
        }
    }

    void SerializedMPOMDP::setupObservationSpace(std::shared_ptr<MPOMDPInterface> mpomdp)
    {
        // Set default joint observation : "No Observation"
        auto empty_observation = std::make_shared<Joint<std::shared_ptr<Observation>>>();

        // Create the Default Joint Observation
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            empty_observation->push_back(sdm::NO_OBSERVATION);
        }

        // Add the information in the variable empty_serial_observation
        this->empty_serial_observation = empty_observation;
        // Create the Serial Observation Space
        std::vector<std::shared_ptr<DiscreteSpace>> all_observation_space;

        std::shared_ptr<DiscreteSpace> space;
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            //If it's the last agent, then the serial observation space correspond to the mpomdp observation space, else it's the empty observation
            if (this->isLastAgent(ag_id))
            {
                space = std::static_pointer_cast<DiscreteSpace>(mpomdp->getObservationSpace(0));
            }
            else
            {
                space = std::make_shared<DiscreteSpace>(std::vector<std::shared_ptr<Item>>({this->empty_serial_observation}));
            }
            all_observation_space.push_back(space);
        }
        this->serialized_observation_space_ = all_observation_space;
    }

    void SerializedMPOMDP::setReachableObservationSpace(std::shared_ptr<MPOMDPInterface> mpomdp)
    {
        auto observation_dynamics = std::make_shared<TabularObservationDynamicsSAS>();
        // Creation of all reachable Observation Space

        // Go over serial states
        for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
        {
            for (const auto &state : *this->getStateSpace(agent_id))
            {
                std::shared_ptr<SerializedState> serialized_state = std::dynamic_pointer_cast<SerializedState>(state);

                // Go over serial action
                for (const auto action_tmp : *this->getActionSpace(agent_id))
                {
                    auto serial_action = action_tmp->toAction();

                    // Go over joint_obs
                    for (const auto next_state : this->getReachableStates(state->toState(), serial_action, 0))
                    {
                        std::shared_ptr<SerializedState> next_serialized_state = std::dynamic_pointer_cast<SerializedState>(next_state);

                        //Update action
                        auto joint_action(serialized_state->getAction());
                        joint_action.push_back(serial_action);

                        std::set<std::shared_ptr<Observation>> all_obs;

                        // Insert the next reachable Observation State
                        if (next_serialized_state->getCurrentAgentId() == 0)
                        {
                            for (const auto obs : mpomdp->getReachableObservations(serialized_state->getHiddenState(), this->getPointeurJointAction(joint_action), next_serialized_state->getHiddenState(), 0))
                            {
                                // all_obs.insert(obs);
                                observation_dynamics->setReachableObservations(state->toState(), serial_action, next_state, obs);
                            }
                        }
                        else
                        {
                            // all_obs.insert(this->empty_serial_observation);
                            observation_dynamics->setReachableObservations(state->toState(), serial_action, next_state, this->empty_serial_observation);
                        }
                    }
                }
            }
        }
        this->obs_dynamics_ = observation_dynamics;
    }

    std::shared_ptr<Joint<std::shared_ptr<Observation>>> SerializedMPOMDP::getDefaultObservation() const
    {
        return std::dynamic_pointer_cast<Joint<std::shared_ptr<Observation>>>(this->empty_serial_observation);
    }

}