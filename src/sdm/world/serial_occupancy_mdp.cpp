#include <sdm/world/serial_occupancy_mdp.hpp>
#include <sdm/core/state/serialized_state.hpp>

namespace sdm
{

    SerialOccupancyMDP::SerialOccupancyMDP()
    {
    }

    SerialOccupancyMDP::SerialOccupancyMDP(const std::shared_ptr<SerialMPOMDPInterface> &underlying_dpomdp,
                                           number memory,
                                           bool compression,
                                           bool store_states,
                                           bool store_actions,
                                           int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, compression, store_states, store_actions, batch_size)
    {
    }

    number SerialOccupancyMDP::getAgentId(number t) const
    {
        return this->getUnderlyingSerializedMMDP()->getAgentId(t);
    }

    bool SerialOccupancyMDP::isLastAgent(number t) const
    {
        return this->getUnderlyingSerializedMMDP()->isLastAgent(t);
    }

    double SerialOccupancyMDP::getDiscount(number t) const
    {
        return this->getUnderlyingSerializedMMDP()->getDiscount(t);
    }

    std::shared_ptr<SerialMMDPInterface> SerialOccupancyMDP::getUnderlyingSerializedMMDP() const
    {
        return std::dynamic_pointer_cast<SerialMMDPInterface>(this->getUnderlyingMDP());
    }

    std::shared_ptr<SerialMPOMDPInterface> SerialOccupancyMDP::getUnderlyingSerializedMPOMDP() const
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

    // std::shared_ptr<State> SerialOccupancyMDP::nextStateSerialStep(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, number t) const
    // {
    //     auto occupancy_state = ostate->toOccupancyState();
    //     auto decision_rule = action->toDecisionRule();

    //     OccupancyMDP::PASSAGE_IN_NEXT_STATE++;
    //     OccupancyMDP::MEAN_SIZE_STATE += occupancy_state->getFullyUncompressedOccupancy()->getStates().size();

    //     // The new fully uncompressed occupancy state
    //     std::shared_ptr<OccupancyStateInterface> fully_uncompressed_next_occupancy_state = std::make_shared<OccupancyState>(occupancy_state);
    //     // The new one step left occupancy state
    //     std::shared_ptr<OccupancyStateInterface> one_step_left_compressed_next_occupancy_state = std::make_shared<OccupancyState>(occupancy_state);

    //     // For each joint history in the support of the fully uncompressed occupancy state
    //     for (const auto &joint_history : occupancy_state->getFullyUncompressedOccupancy()->getJointHistories())
    //     {
    //         fully_uncompressed_next_occupancy_state->getBeliefAt(joint_history);
    //         // Apply the joint decision rule at joint_history to get the joint_action
    //         auto compressed_joint_history = occupancy_state->getCompressedJointHistory(joint_history);
    //         auto indiv_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), compressed_joint_history, decision_rule, t);

    //         // Compute next belief
    //         auto next_belief = this->getUnderlyingBeliefMDP()->nextBelief(fully_uncompressed_next_occupancy_state->getBeliefAt(joint_history), indiv_action, sdm::DEFAULT_OBSERVATION, t);

    //         double proba = occupancy_state->getFullyUncompressedOccupancy()->getProbability(joint_history);

    //         // Build fully uncompressed occupancy state
    //         fully_uncompressed_next_occupancy_state->addProbability(joint_history->toJointHistory(), next_belief->toBelief(), proba);

    //         // Update the probability of being in this next history (for the one step left uncompressed occupancy state)
    //         one_step_left_compressed_next_occupancy_state->addProbability(compressed_joint_history->toJointHistory(), next_belief->toBelief(), proba);
    //     }

    //     fully_uncompressed_next_occupancy_state->finalize();
    //     one_step_left_compressed_next_occupancy_state->finalize();

    //     return std::make_pair(fully_uncompressed_next_occupancy_state, one_step_left_compressed_next_occupancy_state);
    // }

    Pair<std::shared_ptr<State>, std::shared_ptr<State>> SerialOccupancyMDP::computeExactNextState(const std::shared_ptr<State> &ostate, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // if (!this->isLastAgent(t))
        // {
        //     return this->nextStateSerialStep(ostate, action, t);
        // }
        // else
        // {
            return OccupancyMDP::computeExactNextState(ostate, action, observation, t);
        // }
    }

    double SerialOccupancyMDP::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        // 
        return (!this->isLastAgent(t)) ? 0. : OccupancyMDP::getReward(occupancy_state, decision_rule, t);
    }
} // namespace sdm
