#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/core/state/serial_state.hpp>

namespace sdm
{

    SerialOccupancyMDP::SerialOccupancyMDP()
    {
    }

    SerialOccupancyMDP::SerialOccupancyMDP(const std::shared_ptr<SerialMPOMDPInterface> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, compression, store_states, store_actions, batch_size)
    {
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
        return std::dynamic_pointer_cast<SerialMMDPInterface>(this->getUnderlyingMDP());
    }

    std::shared_ptr<SerialMPOMDPInterface> SerialOccupancyMDP::getUnderlyingSerialMPOMDP() const
    {
        return std::dynamic_pointer_cast<SerialMPOMDPInterface>(this->getUnderlyingMDP());
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

    std::shared_ptr<Action> SerialOccupancyMDP::applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const
    {

        // Transform in serial occupancy state
        auto serial_ostate = std::dynamic_pointer_cast<OccupancyState>(ostate);

        // Get the selected joint action
        auto action = std::static_pointer_cast<DeterministicDecisionRule>(decision_rule)->act(joint_history->getIndividualHistory(this->getAgentId(t)));
        return action;
    }

    bool SerialOccupancyMDP::doCompression(number t) const
    {
        return (OccupancyMDP::doCompression(t) /* && this->getAgentId(t) == 0*/);
    }

    double SerialOccupancyMDP::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        return (!this->isLastAgent(t)) ? 0. : OccupancyMDP::getReward(occupancy_state, decision_rule, t);
    }
} // namespace sdm
