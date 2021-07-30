#include <sdm/world/hierarchical_mpomdp.hpp>

namespace sdm
{
    HierarchicalMPOMDP::HierarchicalMPOMDP(const std::shared_ptr<MPOMDPInterface> &mpomdp) : TransformedMPOMDP(mpomdp)
    {
        this->setupObservationSpace();
    }

    std::shared_ptr<Space> HierarchicalMPOMDP::getObservationSpace(number) const
    {
        return this->joint_observation_space_;
    }

    std::shared_ptr<Space> HierarchicalMPOMDP::getObservationSpace(number agent_id, number t) const
    {
        return this->indiv_observation_spaces_.get(agent_id);
    }

    std::set<std::shared_ptr<Observation>> HierarchicalMPOMDP::getReachableObservations(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const
    {
        return this->mpomdp_->getReachableObservations(state, action, next_state, t);
    }

    double HierarchicalMPOMDP::getObservationProbability(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, const std::shared_ptr<Observation> &observation, number t) const
    {
        return this->mpomdp_->getObservationProbability(state, action, next_state, this->getClassicObservation(observation), t);
    }

    std::shared_ptr<Observation> HierarchicalMPOMDP::getClassicObservation(const std::shared_ptr<Observation> &hierarchical_joint_observation) const
    {
        return this->bimap_classic_to_hierarchical_obs.right.at(hierarchical_joint_observation);
    }

    std::shared_ptr<Observation> HierarchicalMPOMDP::getHierarchicalObservation(const std::shared_ptr<Observation> &classic_joint_observation) const
    {
        return this->bimap_classic_to_hierarchical_obs.left.at(classic_joint_observation);
    }

    // std::shared_ptr<Space> HierarchicalMPOMDP::getSubordinateActionSpace(number agent_id) const
    // {
    // }

    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> HierarchicalMPOMDP::step(std::shared_ptr<Action> action)
    {
        auto [obs, rewards, is_done] = this->mpomdp_->step(action);
        return {this->getHierarchicalObservation(obs), rewards, is_done};
    }
    std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> HierarchicalMPOMDP::step(std::shared_ptr<Action> action, bool increment_timestep)
    {
        auto [obs, rewards, is_done] = this->mpomdp_->step(action, increment_timestep);
        return {this->getHierarchicalObservation(obs), rewards, is_done};
    }

    void HierarchicalMPOMDP::setupObservationSpace()
    {

        using obs_type = Joint<std::shared_ptr<Observation>>;
        // For each agent, keep all possible individual hierarchical observations
        Joint<std::unordered_map<obs_type, std::shared_ptr<obs_type>>> map_new_obs_to_ptr;

        // List of all possible joint hierarchical observations
        std::vector<std::shared_ptr<Observation>> list_possible_joint_hierarchical_observation;
        // Initialize for each agent
        for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
        {
            map_new_obs_to_ptr.push_back({});
        }

        // For all joint observations
        for (const auto &joint_observation : *this->mpomdp_->getObservationSpace(0))
        {
            // Instanciate new joint observation
            auto joint_hierarchical_observation = std::make_shared<Joint<std::shared_ptr<Observation>>>();
            // Instanciate temporary joint observation of agent 0:i
            Joint<std::shared_ptr<Observation>> tmp;
            for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
            {
                // Get observation of agent i
                auto indiv_observation = std::static_pointer_cast<Joint<std::shared_ptr<Observation>>>(joint_observation)->get(agent_id);
                // Add it to the temporary structure
                tmp.push_back(indiv_observation);
                if (map_new_obs_to_ptr.at(agent_id).find(tmp) == map_new_obs_to_ptr.at(agent_id).end())
                {
                    auto indiv_h_obs = std::make_shared<Joint<std::shared_ptr<Observation>>>(tmp);
                    map_new_obs_to_ptr.at(agent_id).emplace(tmp, indiv_h_obs);
                }
                // Add indiv hierarchical observation of agent i in the joint observation
                joint_hierarchical_observation->push_back(map_new_obs_to_ptr.at(agent_id).at(tmp));
            }
            list_possible_joint_hierarchical_observation.push_back(joint_hierarchical_observation);
            // Add mapping from basic joint observation to hierarchical joint observation
            this->bimap_classic_to_hierarchical_obs.insert(obs_bimap_value(joint_observation->toObservation(), joint_hierarchical_observation));
        }
        this->joint_observation_space_ = std::make_shared<DiscreteSpace>(list_possible_joint_hierarchical_observation);

        // Setup individual observation spaces
        for (number agent_id = 0; agent_id < this->getNumAgents(); agent_id++)
        {
            std::vector<std::shared_ptr<Observation>> list_indiv_obs;
            for (const auto &pair_obs_pobs : map_new_obs_to_ptr.at(agent_id))
            {
                list_indiv_obs.push_back(pair_obs_pobs.second);
            }
            this->indiv_observation_spaces_.push_back(std::make_shared<DiscreteSpace>(list_indiv_obs));
        }
    }

} // namespace sdm