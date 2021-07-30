#pragma once

#include <sdm/world/occupancy_mdp.hpp>
#include <sdm/world/hierarchical_mpomdp.hpp>

#include <sdm/core/state/jhistory_tree.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    class HierarchicalOccupancyMDP : public OccupancyMDP
    {
    public:
        HierarchicalOccupancyMDP();
        HierarchicalOccupancyMDP(const std::shared_ptr<HierarchicalMPOMDP> &hierarchical_mpomdp, number memory = -1, bool compression = true, bool store_states = true, bool store_actions = true, bool generate_action_spaces = false, int batch_size = 0);

        /**
         * @brief Get the identifier of the agent at the bottom of the hierarchy 
         * 
         * @return number the id of the agent
         */
        number getLowLevelAgentID();

        std::shared_ptr<Space> getObservationSpace(number t);
        virtual std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);
        bool checkCompatibility(const std::shared_ptr<Observation> &joint_observation, const std::shared_ptr<Observation> &observation);

    protected:
        std::shared_ptr<Action> current_action_;
        /** @brief the identifier of the agent at the last level of the hierarchy */
        number low_level_agent_id_;
        Pair<std::shared_ptr<State>, double> computeNextStateAndProbability(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
      };
} // namespace sdm
