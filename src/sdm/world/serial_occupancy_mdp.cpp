#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

    SerialOccupancyMDP::SerialOccupancyMDP()
    {
    }

    SerialOccupancyMDP::SerialOccupancyMDP(const std::shared_ptr<SerialMPOMDPInterface> &underlying_mpomdp, number memory, bool store_states, bool store_actions, int batch_size)
        : OccupancyMDP(underlying_mpomdp, memory, store_states, store_actions, batch_size), underlying_serial_mpomdp(underlying_mpomdp)
    {
    }

    SerialOccupancyMDP::SerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, Config config) : OccupancyMDP(dpomdp, config)
    {
        this->underlying_serial_mpomdp = std::dynamic_pointer_cast<SerialMPOMDPInterface>(dpomdp);
    }

    SerialOccupancyMDP::SerialOccupancyMDP(Config config) : OccupancyMDP(config)
    {
        this->underlying_serial_mpomdp = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->getUnderlyingMPOMDP());
    }

    number SerialOccupancyMDP::getAgentId(number t) const
    {
        return this->getUnderlyingSerialMMDP()->getAgentId(t);
    }

    bool SerialOccupancyMDP::isLastAgent(number t) const
    {
        return this->getUnderlyingSerialMMDP()->isLastAgent(t);
    }

    double SerialOccupancyMDP::getDiscount(number t) const
    {
        return this->getUnderlyingSerialMMDP()->getDiscount(t);
    }

    std::shared_ptr<SerialMMDPInterface> SerialOccupancyMDP::getUnderlyingSerialMMDP() const
    {
        return this->underlying_serial_mpomdp;
    }

    std::shared_ptr<SerialMPOMDPInterface> SerialOccupancyMDP::getUnderlyingSerialMPOMDP() const
    {
        return this->underlying_serial_mpomdp;
    }

    std::shared_ptr<Space> SerialOccupancyMDP::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // Transform in serial occupancy state
        auto serial_ostate = ostate->toOccupancyState();
        // Get individual histories of agent i.
        std::set<std::shared_ptr<HistoryInterface>> individual_histories = serial_ostate->toOccupancyState()->getIndividualHistories(this->getAgentId(t));
        // Get individual history space of agent i.
        std::shared_ptr<Space> individual_history_space = std::make_shared<DiscreteSpace>(sdm::tools::set2vector(individual_histories));
        // Get action space of agent i.
        std::shared_ptr<Space> individual_action_space = this->getUnderlyingProblem()->getActionSpace(t);
        // Get individual ddr of agent i.
        std::shared_ptr<Space> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule>>(individual_history_space, individual_action_space, this->store_actions_);

        return individual_ddr_space;
    }
    
    double SerialOccupancyMDP::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        return (!this->isLastAgent(t)) ? 0. : OccupancyMDP::getReward(occupancy_state, decision_rule, t);
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

    std::shared_ptr<State> SerialOccupancyMDP::getDecisionRuleInput(const std::shared_ptr<JointHistoryInterface> &jhistory, number t) const
    {
        return jhistory->getIndividualHistory(this->getAgentId(t));
    }

} // namespace sdm
