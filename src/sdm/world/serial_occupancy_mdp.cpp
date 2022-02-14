#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

    SerialOccupancyMDP::SerialOccupancyMDP()
    {
        this->setupEmptyObservation();
    }

    SerialOccupancyMDP::SerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_mpomdp, number memory, bool store_states, bool store_actions, int batch_size)
        : BaseOccupancyMDP<OccupancyStateSerial>(underlying_mpomdp, memory, store_states, store_actions, batch_size), underlying_serial_mpomdp(underlying_mpomdp)
    {
        this->setupEmptyObservation();
    }

    SerialOccupancyMDP::SerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, Config config) : BaseOccupancyMDP<OccupancyStateSerial>(dpomdp, config)
    {
        this->setupEmptyObservation();
        this->underlying_serial_mpomdp = std::dynamic_pointer_cast<SerialMPOMDPInterface>(dpomdp);
    }

    SerialOccupancyMDP::SerialOccupancyMDP(Config config) : BaseOccupancyMDP<OccupancyStateSerial>(config)
    {
        this->setupEmptyObservation();
        this->underlying_serial_mpomdp = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->getUnderlyingMPOMDP());
    }

    number SerialOccupancyMDP::getAgentId(number t) const
    {
        return (t % this->underlying_serial_mpomdp->getNumAgents());
    }

    bool SerialOccupancyMDP::isLastAgent(number t) const
    {
        return (this->getAgentId(t) == (this->underlying_serial_mpomdp->getNumAgents() - 1));
    }

    double SerialOccupancyMDP::getDiscount(number t) const
    {
        return this->getUnderlyingSerialMMDP()->getDiscount(t);
    }

    std::shared_ptr<MMDPInterface> SerialOccupancyMDP::getUnderlyingSerialMMDP() const
    {
        return this->underlying_serial_mpomdp;
    }

    std::shared_ptr<MPOMDPInterface> SerialOccupancyMDP::getUnderlyingSerialMPOMDP() const
    {
        return this->underlying_serial_mpomdp;
    }

    std::shared_ptr<Space> SerialOccupancyMDP::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // Transform in serial occupancy state
        auto serial_ostate = ostate->toOccupancyState();
        // Get individual histories of agent i.
        std::set<std::shared_ptr<HistoryInterface>> individual_histories = serial_ostate->getIndividualHistories(this->getAgentId(t));
        // Get individual history space of agent i.
        std::shared_ptr<Space> individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_histories));
        // Get action space of agent i.
        std::shared_ptr<Space> individual_action_space = this->getUnderlyingMPOMDP()->getActionSpace(this->getAgentId(t), t);
        // Get individual ddr of agent i.
        std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, this->store_actions_);

        return individual_ddr_space;
    }
    double SerialOccupancyMDP::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        return (!this->isLastAgent(t)) ? 0. : BaseOccupancyMDP<OccupancyStateSerial>::getReward(occupancy_state, decision_rule, t);
    }

    std::shared_ptr<Action> SerialOccupancyMDP::computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t)
    {
        // Input states for the a of agent.
        std::vector<std::shared_ptr<Item>> inputs;
        // Outputed actions for each of these.
        std::vector<std::shared_ptr<Item>> outputs;

        number agentID = this->getAgentId(t);

        for (const auto &individual_history : ostate->getIndividualHistories(agentID))
        {
            inputs.push_back(individual_history);
            outputs.push_back(this->getUnderlyingMPOMDP()->getActionSpace(agentID, t)->sample());
        }
        return std::make_shared<DeterministicDecisionRule>(inputs, outputs);
    }

    void SerialOccupancyMDP::setupEmptyObservation() {
        // Set default joint observation : "No Observation"
        this->empty_observation = std::make_shared<JointObservation>();

        // Create the Default Joint Observation
        for (number ag_id = 0; ag_id < this->underlying_serial_mpomdp->getNumAgents(); ag_id++)
        {
            this->empty_observation->push_back(sdm::NO_OBSERVATION);
        }
    }


    std::shared_ptr<JointObservation> SerialOccupancyMDP::getDefaultObservation() const
    {
        return this->empty_observation;
    }


} // namespace sdm
