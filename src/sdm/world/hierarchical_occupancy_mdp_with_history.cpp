#include <sdm/world/hierarchical_occupancy_mdp_with_history.hpp>

namespace sdm
{

    HierarchicalOccupancyMDPWithHistory::HierarchicalOccupancyMDPWithHistory()
    {
    }

    HierarchicalOccupancyMDPWithHistory::HierarchicalOccupancyMDPWithHistory(const std::shared_ptr<HierarchicalMPOMDP> &hierarchical_mpomdp, number memory, bool compression, bool store_states, bool store_actions, bool generate_action_spaces, int batch_size)
        : HierarchicalOccupancyMDP(hierarchical_mpomdp, memory, compression, store_states, store_actions, generate_action_spaces, batch_size)
    {
        
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> HierarchicalOccupancyMDPWithHistory::step(std::shared_ptr<Action> action)
    {
        auto [s, rewards, is_done] = HierarchicalOccupancyMDP::step(action);

        std::shared_ptr<PrivateHierarchicalOccupancyStateJointHistoryPair> s_o = std::make_shared<PrivateHierarchicalOccupancyStateJointHistoryPair>(std::make_pair(s->toState()->toOccupancyState(), this->current_history_->toJointHistory()));

        return std::make_tuple(s_o, rewards, is_done);
    }

    std::shared_ptr<Observation> HierarchicalOccupancyMDPWithHistory::reset()
    {
        OccupancyMDP::reset();

        std::shared_ptr<PrivateHierarchicalOccupancyStateJointHistoryPair> s_o = std::make_shared<PrivateHierarchicalOccupancyStateJointHistoryPair>(std::make_pair(this->current_state_->toState()->toOccupancyState(), this->current_history_->toJointHistory()));

        return s_o;
    }

    double HierarchicalOccupancyMDPWithHistory::getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t)
    {
        // auto joint_label = this->getJointLabel(this->current_history_, occupancy_state);
        auto belief = occupancy_state->toOccupancyState()->getBeliefAt(this->current_history_->toJointHistory());
        auto joint_action = this->applyDecisionRule(occupancy_state->toOccupancyState(), this->current_history_->toJointHistory(), decision_rule, t);
        return this->getUnderlyingBeliefMDP()->getReward(belief, joint_action, t);
    }

    std::shared_ptr<Action> HierarchicalOccupancyMDPWithHistory::getRandomAction(const std::shared_ptr<Observation> &ostate, number t)
    {
        // std::cout << "HierarchicalOccupancyMDPWithHistory::getRandomAction() " << std::endl;

        auto s = std::dynamic_pointer_cast<PrivateHierarchicalOccupancyStateJointHistoryPair>(ostate)->first;

        return HierarchicalOccupancyMDP::getRandomAction(s->toState()->toObservation(), t);
    }

    // std::shared_ptr<HistoryInterface> HierarchicalOccupancyMDPWithHistory::getJointLabel(const std::shared_ptr<HistoryInterface> &joint_history, const std::shared_ptr<State> &occupancy_state)
    // {
    //     std::shared_ptr<JointHistoryInterface> joint_label = std::make_shared<JointHistoryTree>();
    //     int i = 0;
    //     for (const auto &individual_history : joint_history->toJointHistory()->getIndividualHistories())
    //     {
    //         joint_label->addIndividualHistory(occupancy_state->toOccupancyState()->getLabel(individual_history, i++));
    //     }
    //     for (const auto &joint_history : occupancy_state->toOccupancyState()->getJointHistories())
    //     {
    //         if (*std::dynamic_pointer_cast<JointHistoryTree>(joint_history) == *std::dynamic_pointer_cast<JointHistoryTree>(joint_label))
    //             return joint_history;
    //     }
    // }

} // namespace sdm
