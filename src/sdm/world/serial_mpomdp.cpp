#include <sdm/world/serial_mpomdp.hpp>
#include <sdm/core/observation/default_observation.hpp>
#include <sdm/world/registry.hpp>

namespace sdm
{
    SerialMPOMDP::SerialMPOMDP()
    {
    }

    SerialMPOMDP::SerialMPOMDP(Config config)
        : SerialMPOMDP(std::dynamic_pointer_cast<MPOMDPInterface>(sdm::world::createFromConfig(config)), config)
    {
    }

    SerialMPOMDP::SerialMPOMDP(std::shared_ptr<MPOMDPInterface> mpomdp, Config) : SerialMMDP(mpomdp), mpomdp(mpomdp)
    {
        setupObservationSpace(mpomdp);
        setReachableObservationSpace(mpomdp);
    }

    std::shared_ptr<ObservationSpace> SerialMPOMDP::getObservationSpace(number t) const
    {
        return this->serial_observation_space_.at(this->getAgentId(t));
    }

    std::shared_ptr<ObservationSpace> SerialMPOMDP::getObservationSpace(number agent_id, number t) const
    {
        return this->getObservationSpace(t);
    }

    std::set<std::shared_ptr<Observation>> SerialMPOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->obs_dynamics_->getReachableObservations(state, action, next_state, t);
    }

    double SerialMPOMDP::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        auto serial_state = state->toSerial();
        auto next_serial_state = next_state->toSerial();

        auto all_action = serial_state->getAction();
        all_action.push_back(action);

        auto indiv_observation_space = std::static_pointer_cast<DiscreteObservationSpace>(this->getObservationSpace(serial_state->getCurrentAgentId()));
        auto index = indiv_observation_space->find(observation);

        if (index != -1 && next_serial_state->getCurrentAgentId() == 0 && serial_state->getCurrentAgentId() == this->getNumAgents() - 1 && indiv_observation_space->getItem(index))
        {
            // The probability is the same of the decpomdp, if the condition are verified
            return this->mpomdp->getObservationProbability(serial_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serial_state->getHiddenState(), observation, t);
        }
        else
        {
            // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
            // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
            return (this->empty_serial_observation == observation && serial_state->getHiddenState() == next_serial_state->getHiddenState() && next_serial_state->getAction() == all_action) ? 1 : 0;
        }
    }

    double SerialMPOMDP::getDynamics(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        auto serial_state = state->toSerial();
        auto next_serial_state = next_state->toSerial();

        auto all_action = serial_state->getAction();
        all_action.push_back(action);

        // auto&& observation_space = this->serial_observation_space_.at(serial_state->getCurrentAgentId());

        // auto&& index = observation_space->find(observation);

        if (this->serial_observation_space_.at(serial_state->getCurrentAgentId())->contains(observation) && next_serial_state->getCurrentAgentId() == 0 && serial_state->getCurrentAgentId() == this->getNumAgents() - 1 /*  && observation_space->getItem(index) */)
        {
            // The probability is the same of the decpomdp, if the condition are verified
            return this->mpomdp->getDynamics(serial_state->getHiddenState(), this->getPointeurJointAction(all_action), next_serial_state->getHiddenState(), observation, t);
        }
        else
        {
            // If it is a intermediate agent, the probability is worth 1 only if : the joint_obs is the empty observation,
            // the currentand next hidden state are equal, the action of the next serial state is equal to the action of current serial state + serial action
            return (this->empty_serial_observation == observation && serial_state->getHiddenState() == next_serial_state->getHiddenState() && next_serial_state->getAction() == all_action) ? 1. : 0.;
        }
    }

    void SerialMPOMDP::setupObservationSpace(std::shared_ptr<MPOMDPInterface> mpomdp)
    {
        // Set default joint observation : "No Observation"
        auto empty_observation = std::make_shared<JointObservation>();

        // Create the Default Joint Observation
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            empty_observation->push_back(sdm::NO_OBSERVATION);
        }

        // Add the information in the variable empty_serial_observation
        this->empty_serial_observation = empty_observation;
        // Create the Serial Observation Space
        std::vector<std::shared_ptr<DiscreteObservationSpace>> all_observation_space;

        std::shared_ptr<DiscreteObservationSpace> space;
        for (number ag_id = 0; ag_id < this->getNumAgents(); ag_id++)
        {
            // If it's the last agent, then the serial observation space correspond to the mpomdp observation space, else it's the empty observation
            if (this->isLastAgent(ag_id))
                space = std::static_pointer_cast<DiscreteObservationSpace>(mpomdp->getObservationSpace(0));
            else
                space = std::make_shared<DiscreteObservationSpace>(std::vector<std::shared_ptr<Observation>>({this->empty_serial_observation}));
            all_observation_space.push_back(space);
        }
        this->serial_observation_space_ = all_observation_space;
    }

    void SerialMPOMDP::setReachableObservationSpace(std::shared_ptr<MPOMDPInterface> mpomdp)
    {
        auto observation_dynamics = std::make_shared<TabularObservationDynamicsSAS>();
        // Creation of all reachable Observation Space

        // Go over serial states
        for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
        {
            auto state_end_iter = this->getStateSpace(agent_id)->end();
            for (auto state_iter = this->getStateSpace(agent_id)->begin(); !state_iter->equal(state_end_iter); state_iter = state_iter->next())
            {
                auto state = state_iter->getCurrent();
                auto serial_state = state->toSerial();

                auto end_iter = this->getActionSpace(agent_id)->end();
                for (auto iter = this->getActionSpace(agent_id)->begin(); !iter->equal(end_iter); iter->next())
                {
                    auto serial_action = iter->getCurrent();
                    // Go over joint_obs
                    for (const auto next_state : this->getReachableStates(state, serial_action, 0))
                    {
                        auto next_serial_state = next_state->toSerial();

                        // Update action
                        auto joint_action(serial_state->getAction());
                        joint_action.push_back(serial_action);

                        std::set<std::shared_ptr<Observation>> all_obs;

                        // Insert the next reachable Observation State
                        if (next_serial_state->getCurrentAgentId() == 0)
                        {
                            for (const auto obs : mpomdp->getReachableObservations(serial_state->getHiddenState(), this->getPointeurJointAction(joint_action), next_serial_state->getHiddenState(), 0))
                            {
                                // all_obs.insert(obs);
                                observation_dynamics->setReachableObservations(state, serial_action, next_state, obs);
                            }
                        }
                        else
                        {
                            // all_obs.insert(this->empty_serial_observation);
                            observation_dynamics->setReachableObservations(state, serial_action, next_state, this->empty_serial_observation);
                        }
                    }
                }
            }
        }
        this->obs_dynamics_ = observation_dynamics;
    }

    std::shared_ptr<JointObservation> SerialMPOMDP::getDefaultObservation() const
    {
        return std::dynamic_pointer_cast<JointObservation>(this->empty_serial_observation);
    }

}