#include <memory>
#include <sdm/world/hierarchical_occupancy_mdp.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>

namespace sdm
{
    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP()
    {
    }

    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP(const std::shared_ptr<HierarchicalMPOMDP> &underlying_dpomdp, number memory, bool compression, bool store_states, bool store_actions, int batch_size)
        : OccupancyMDP(underlying_dpomdp, memory, compression, store_states, store_actions, batch_size)
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
        // Compute next state
        std::shared_ptr<State> next_state = this->computeNextState(belief, action, observation, t);

        // Compute the coefficient of normalization (eta)
        double eta = next_state->toBelief()->norm_1();

        // Normalize to belief
        next_state->toBelief()->normalizeBelief(eta);
        
        // Return the pair next belief / proba of the transition in this belief
        return {next_state->toBelief(), eta};
    }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> HierarchicalOccupancyMDP::step(std::shared_ptr<Action> action)
    {
        // Compute reward
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);

        double cumul = 0.0, prob = 0.0;
        std::shared_ptr<State> candidate_state_ = nullptr;

        // Get an random number between 0 and 1
        double epsilon = std::rand() / (double(std::RAND_MAX));

        // Go over all observations of the lower-level agent
        for(auto obs_n : this->getUnderlyingMPOMDP()->getObservationSpace(this->getLowLevelAgentID(), this->step_))
        {
            std::tie(candidate_state_, prob) = computeNextStateAndProbability(this->current_state_, action, obs_n, this->step_);

            cumul += prob;
            if (epsilon < cumul)
            {
                this->step_++;
                this->current_state_ =  candidate_state_;
                return std::make_tuple(this->current_state_, std::vector<double>(this->getUnderlyingMPOMDP()->getNumAgents(), occupancy_reward), (this->step >= this->getUnderlyingMPOMDP()->getHorizon()));
            }
        }
    }
} // namespace sdm
