#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

    template <typename TSerialOState>
    BaseSerialOccupancyMDP<TSerialOState>::BaseSerialOccupancyMDP()
    {
        this->setupEmptyObservation();
    }

    template <typename TSerialOState>
    BaseSerialOccupancyMDP<TSerialOState>::BaseSerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_mpomdp, int memory, bool store_states, bool store_actions, int batch_size)
        : BaseOccupancyMDP<TSerialOState>(underlying_mpomdp, memory, store_states, store_actions, batch_size), underlying_serial_mpomdp(underlying_mpomdp)
    {
        this->setupEmptyObservation();
        if (sdm::isInstanceOf<SerialProblemInterface>(underlying_serial_mpomdp))
            this->horizon = underlying_serial_mpomdp->getHorizon();
        else
            this->horizon = underlying_serial_mpomdp->getHorizon() * underlying_serial_mpomdp->getNumAgents();
    }

    template <typename TSerialOState>
    BaseSerialOccupancyMDP<TSerialOState>::BaseSerialOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, Config config) : BaseOccupancyMDP<TSerialOState>(dpomdp, config)
    {
        this->setupEmptyObservation();
        this->underlying_serial_mpomdp = std::dynamic_pointer_cast<SerialMPOMDPInterface>(dpomdp);
    }

    template <typename TSerialOState>
    BaseSerialOccupancyMDP<TSerialOState>::BaseSerialOccupancyMDP(Config config) : BaseOccupancyMDP<TSerialOState>(config)
    {
        this->setupEmptyObservation();
        this->underlying_serial_mpomdp = std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->getUnderlyingMPOMDP());
    }

    template <typename TSerialOState>
    number BaseSerialOccupancyMDP<TSerialOState>::getHorizon() const
    {
        return this->horizon;
    }

    template <typename TSerialOState>
    number BaseSerialOccupancyMDP<TSerialOState>::getAgentId(number t) const
    {
        return (t % this->underlying_serial_mpomdp->getNumAgents());
    }

    template <typename TSerialOState>
    bool BaseSerialOccupancyMDP<TSerialOState>::isLastAgent(number t) const
    {
        return (this->getAgentId(t) == (this->underlying_serial_mpomdp->getNumAgents() - 1));
    }

    template <typename TSerialOState>
    double BaseSerialOccupancyMDP<TSerialOState>::getDiscount(number t) const
    {
        return this->isLastAgent(t) ? this->underlying_serial_mpomdp->getDiscount(this->getAgentId(t)) : 1.0;
    }

    template <typename TSerialOState>
    std::shared_ptr<MMDPInterface> BaseSerialOccupancyMDP<TSerialOState>::getUnderlyingSerialMMDP() const
    {
        return this->underlying_serial_mpomdp;
    }

    template <typename TSerialOState>
    std::shared_ptr<MPOMDPInterface> BaseSerialOccupancyMDP<TSerialOState>::getUnderlyingSerialMPOMDP() const
    {
        return this->underlying_serial_mpomdp;
    }

    template <typename TSerialOState>
    std::shared_ptr<ActionSpace> BaseSerialOccupancyMDP<TSerialOState>::computeActionSpaceAt(const std::shared_ptr<State> &ostate, number t)
    {
        // Transform in serial occupancy state
        auto serial_ostate = ostate->toOccupancyState();
        // Get individual histories of agent i.
        std::set<std::shared_ptr<HistoryInterface>> individual_histories = serial_ostate->getIndividualHistories(this->getAgentId(t));
        // Get individual history space of agent i.
        std::shared_ptr<DiscreteSpace<std::shared_ptr<HistoryInterface>>> individual_history_space = std::make_shared<DiscreteSpace<std::shared_ptr<HistoryInterface>>>(sdm::tools::set2vector(individual_histories));

        // Get action space of agent i.
        std::shared_ptr<ActionSpace> individual_action_space = this->getUnderlyingMPOMDP()->getActionSpace(this->getAgentId(t), t);
        // Get individual ddr of agent i.
        std::shared_ptr<ActionSpace> individual_ddr_space = std::make_shared<FunctionSpace<DeterministicDecisionRule, Action>>(individual_history_space, individual_action_space, this->store_actions_);

        return individual_ddr_space;
    }

    template <typename TSerialOState>
    double BaseSerialOccupancyMDP<TSerialOState>::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        return (!this->isLastAgent(t)) ? 0. : BaseOccupancyMDP<TSerialOState>::getReward(occupancy_state, decision_rule, t);
    }

    template <typename TSerialOState>
    std::shared_ptr<Action> BaseSerialOccupancyMDP<TSerialOState>::computeRandomAction(const std::shared_ptr<OccupancyStateInterface> &ostate, number t)
    {
        // Input states for the a of agent.
        std::vector<std::shared_ptr<HistoryInterface>> inputs;
        // Outputed actions for each of these.
        std::vector<std::shared_ptr<Action>> outputs;

        number agentID = this->getAgentId(t);

        for (const auto &individual_history : ostate->getIndividualHistories(agentID))
        {
            inputs.push_back(individual_history);
            outputs.push_back(this->getUnderlyingMPOMDP()->getActionSpace(agentID, t)->sample());
        }
        return std::make_shared<DeterministicDecisionRule>(inputs, outputs);
    }

    template <typename TSerialOState>
    void BaseSerialOccupancyMDP<TSerialOState>::setupEmptyObservation()
    {
        // Set default joint observation : "No Observation"
        this->empty_observation = std::make_shared<JointObservation>();
        // Create the Default Joint Observation
        for (number ag_id = 0; ag_id < this->underlying_serial_mpomdp->getNumAgents(); ag_id++)
        {
            this->empty_observation->push_back(sdm::NO_OBSERVATION);
        }
    }

    template <typename TSerialOState>
    std::shared_ptr<JointObservation> BaseSerialOccupancyMDP<TSerialOState>::getDefaultObservation() const
    {
        return this->empty_observation;
    }

} // namespace sdm
