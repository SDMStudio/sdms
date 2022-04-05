#include <memory>
#include <sdm/world/hierarchical_occupancy_mdp.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>

namespace sdm
{
    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP()
    {
    }

    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP(const std::shared_ptr<HierarchicalMPOMDP> &underlying_dpomdp, int memory, bool store_states, bool store_actions, int batch_size)
        : BaseOccupancyMDP<HierarchicalOccupancyState>(underlying_dpomdp, memory, store_states, store_actions, batch_size)
    {
        this->low_level_agent_id_ = 0;
    }

    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP(const std::shared_ptr<MPOMDPInterface> &dpomdp, Config config): BaseOccupancyMDP<HierarchicalOccupancyState>(dpomdp, config)
    {
        this->low_level_agent_id_ = 0;
    }

    HierarchicalOccupancyMDP::HierarchicalOccupancyMDP(Config config): BaseOccupancyMDP<HierarchicalOccupancyState>(config)
    {
        this->low_level_agent_id_ = 0;
    }

    number HierarchicalOccupancyMDP::getLowLevelAgentID()
    {
        return this->low_level_agent_id_;
    }

    std::shared_ptr<Space> HierarchicalOccupancyMDP::getObservationSpaceAt(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number t)
    {
        return this->getUnderlyingMPOMDP()->getObservationSpace(this->getLowLevelAgentID(), t);
    }

    std::tuple<std::shared_ptr<State>, std::vector<double>, bool> HierarchicalOccupancyMDP::step(std::shared_ptr<Action> action)
    {
        // Compute reward
        double occupancy_reward = this->getReward(this->current_state_, action, this->step_);

        double cumul = 0.0, prob = 0.0;
        std::shared_ptr<State> candidate_state = nullptr;

        // Get a random number between 0 and 1
        double epsilon = std::rand() / (double(RAND_MAX));

        // Go over all observations of the lower-level agent
        for (auto obs_n : *this->getUnderlyingMPOMDP()->getObservationSpace(this->getLowLevelAgentID(), this->step_))
        {
            std::tie(candidate_state, prob) = this->getNextStateAndProba(this->current_state_, action, obs_n->toObservation(), this->step_);

            cumul += prob;
            if (epsilon < cumul)
            {
                this->current_state_ = candidate_state;
                break;
            }
        }

        this->step_++;
        bool is_done = (this->getHorizon() > 0) ? (this->step_ >= this->getHorizon()) : false;
        return std::make_tuple(this->current_state_, std::vector<double>(this->getUnderlyingMPOMDP()->getNumAgents(), occupancy_reward), is_done);
    }
} // namespace sdm
