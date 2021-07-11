#include <sdm/world/private_hierarchical_occupancy_mdp.hpp>

namespace sdm
{

    PrivateHierarchicalOccupancyMDP::PrivateHierarchicalOccupancyMDP()
    {

    }

    PrivateHierarchicalOccupancyMDP::PrivateHierarchicalOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &underlying_dpomdp, number memory, int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, batch_size, false)
    {
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> PrivateHierarchicalOccupancyMDP::step(std::shared_ptr<Action> action)
    {
        auto joint_action = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);
        auto [observation, rewards, is_done]  = this->getUnderlyingProblem()->step(joint_action);
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);
        std::shared_ptr<Observation> observation_n = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(observation)->at(this->getUnderlyingMDP()->getNumAgents() - 1);
        this->current_state_ = this->nextOccupancyState(this->current_state_, action, observation_n, this->step_);
        this->current_history_ = this->getNextHistory(observation);
        this->step_++;
        return std::make_tuple(this->current_state_, std::vector<double>{occupancy_reward, rewards[0]}, is_done);
    }

    
    
} // namespace sdm
