#include <sdm/world/hierarchical_occupancy_mdp.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>

namespace sdm
{
    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP()
    {
    }

    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP(const std::shared_ptr<HierarchicalMPOMDP> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, bool generate_action_spaces, int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, compression, store_states, store_actions, generate_action_spaces, batch_size)
    {
        this->low_level_agent_id_ = 0;
    }

    number HierarchicalOccupancyMDP::getLowLevelAgentID()
    {
        return this->low_level_agent_id_;
    }

    std::shared_ptr<Space> HierarchicalOccupancyMDP::getObservationSpace(number t)
    {
        return this->getUnderlyingMPOMDP()->getObservationSpace(this->getLowLevelAgentID(), t);
    }

    bool HierarchicalOccupancyMDP::checkCompatibility(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Observation> &observation)
    {
        return (std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(joint_observation)->get(this->getLowLevelAgentID()) == observation);
    }

    Pair<std::shared_ptr<State>, double> HierarchicalOccupancyMDP::computeNextStateAndProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t)
    {
        // return BaseBeliefMDP<OccupancyState>::computeNextStateAndProbability(belief, action, observation, t);

        // Compute next state
        std::shared_ptr<State> next_belief = this->computeNextState(belief, action, observation, t);
        std::cout << "Next State" << std::endl;
        std::cout << *next_belief << std::endl;
        // Compute the coefficient of normalization (eta)
        // double eta = next_belief->toBelief()->norm_1();
        // // Normalize to belief
        // next_belief->toBelief()->normalizeBelief(eta);
        // Return the pair next belief / proba of the transition in this belief
        return {next_belief, 1};
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> HierarchicalOccupancyMDP::step(std::shared_ptr<Action> action)
    {
        this->current_action_ = this->applyDecisionRule(this->current_state_->toOccupancyState(), this->current_history_->toJointHistory(), action, this->step_);

        // Do a step on the underlying problem
        auto [observation, rewards, is_done] = this->getUnderlyingProblem()->step(this->current_action_);

        // Compute reward
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);

        std::shared_ptr<Observation> observation_n = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(observation)->at(this->getLowLevelAgentID());

        // Compute next belief
        this->current_state_ = this->nextBelief(this->current_state_, action, observation_n, this->step_);

        this->current_history_ = this->getNextHistory(observation);
        this->step_++;
        return std::make_tuple(this->current_state_, std::vector<double>{occupancy_reward}, is_done);
    }
} // namespace sdm
