#include <sdm/world/private_hierarchical_occupancy_mdp_with_history.hpp>

namespace sdm
{

    PrivateHierarchicalOccupancyMDPWithHistory::PrivateHierarchicalOccupancyMDPWithHistory()
    {
    }

    PrivateHierarchicalOccupancyMDPWithHistory::PrivateHierarchicalOccupancyMDPWithHistory(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, int batch_size)
        : PrivateHierarchicalOccupancyMDP(underlying_dpomdp, memory, compression, store_states, store_actions, batch_size)
    {
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> PrivateHierarchicalOccupancyMDPWithHistory::step(std::shared_ptr<Action> action)
    {
        auto [current_state, rewards, is_done] = PrivateHierarchicalOccupancyMDP::step(action);

        std::shared_ptr<PrivateHierarchicalOccupancyStateJointHistoryPair> state_history = std::make_shared<PrivateHierarchicalOccupancyStateJointHistoryPair>(std::make_pair(current_state->toState()->toOccupancyState(), this->current_history_->toJointHistory()));

        std::shared_ptr<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair> state_history__action = std::make_shared<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(state_history, this->current_action_);

        return std::make_tuple(state_history__action, rewards, is_done);
    }

    std::shared_ptr<Observation> PrivateHierarchicalOccupancyMDPWithHistory::reset()
    {
        OccupancyMDP::reset();

        std::shared_ptr<PrivateHierarchicalOccupancyStateJointHistoryPair> state_history = std::make_shared<PrivateHierarchicalOccupancyStateJointHistoryPair>(std::make_pair(this->current_state_->toState()->toOccupancyState(), this->current_history_->toJointHistory()));

        std::shared_ptr<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair> state_history__action = std::make_shared<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(state_history, nullptr);

        return state_history__action;
    }

    double PrivateHierarchicalOccupancyMDPWithHistory::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        auto joint_label = this->getJointLabel(this->current_history_, occupancy_state);
        auto belief = occupancy_state->toOccupancyState()->getBeliefAt(joint_label->toJointHistory());
        auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), this->current_history_->toJointHistory(), decision_rule, t);
        return this->getUnderlyingBeliefMDP()->getReward(belief, joint_action, t);
    }

    std::shared_ptr<Action> PrivateHierarchicalOccupancyMDPWithHistory::getRandomAction(const std::shared_ptr<Observation> &ostate, number t)
    {
        // std::cout << "PrivateHierarchicalOccupancyMDPWithHistory::getRandomAction() " << std::endl;

        auto s_o = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(ostate)->first;

        return PrivateHierarchicalOccupancyMDP::getRandomAction(s_o->first->toState()->toObservation(), t);
    }

    // std::shared_ptr<Space> PrivateHierarchicalOccupancyMDPWithHistory::getActionSpaceAt(const std::shared_ptr<Observation> &ostate, number t)
    // {
    //     std::cout << "PrivateHierarchicalOccupancyMDPWithHistory::getActionSpaceAt() " << std::endl;
    //     auto state_history = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryJointActionPair>(ostate)->first;
    //     return OccupancyMDP::getActionSpaceAt(state_history->first->toState(), t);
    // }

    std::shared_ptr<HistoryInterface> PrivateHierarchicalOccupancyMDPWithHistory::getJointLabel(const std::shared_ptr<HistoryInterface> &joint_history, const std::shared_ptr<State> &occupancy_state)
    {
        std::shared_ptr<JointHistoryInterface> joint_label = std::make_shared<JointHistoryTree>();
        int i = 0;
        for (const auto &individual_history : joint_history->toJointHistory()->getIndividualHistories())
        {
            joint_label->addIndividualHistory(occupancy_state->toOccupancyState()->getLabel(individual_history, i++));
        }
        for (const auto &joint_history : occupancy_state->toOccupancyState()->getJointHistories())
        {
            if (*std::dynamic_pointer_cast<JointHistoryTree>(joint_history) == *std::dynamic_pointer_cast<JointHistoryTree>(joint_label))
                return joint_history;
        }
    }

} // namespace sdm
