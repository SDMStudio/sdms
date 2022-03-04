#pragma once

#include <sdm/types.hpp>

namespace sdm
{
    class NetworkedDistributedPOMDPInterface : public MPOMDPInterface
    {
        virtual number getNumGroups() const = 0;
        virtual std::vector<int> getGroup(int group_id) const;

        /**
         * @brief Get the state space of a specific agent. 
         * 
         * @param agent_id the identifier of the agent
         * @param t the timestep
         * @return the  
         */
        virtual std::shared_ptr<Space> getLocalStateSpace(number agent_id, number t) const = 0;

        virtual std::set<std::shared_ptr<State>> getLocalReachableStates(number agent_id, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t) const = 0;
        
        virtual double getLocalTransitionProbability(number agent_id, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_state, number t) const = 0;
        
        virtual std::shared_ptr<Distribution<std::shared_ptr<State>>> getLocalStartDistribution(number agent_id) const = 0;

        virtual double getReward(number group_id, const std::shared_ptr<State> &global_state, const std::vector<std::shared_ptr<Action>> &group_action, number t) const = 0;
        virtual double getReward(number group_id, const std::vector<std::shared_ptr<State>> &group_state, const std::vector<std::shared_ptr<Action>> &group_action, number t) const = 0;
    };
} // namespace sdm
