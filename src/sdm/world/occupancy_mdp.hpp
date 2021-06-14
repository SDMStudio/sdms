/**
 * @file occupancy_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/beliefs.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/base/base_occupancy_mdp.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    class OccupancyMDP : public BeliefMDP
    {
    public:
        OccupancyMDP();
        OccupancyMDP(std::string dpomdp_name, number max_history_length = -1);
        OccupancyMDP(std::shared_ptr<POMDPInterface> dpomdp, number max_history_length = -1);

        void initialize(number history_length);
        std::tuple<TState, std::vector<double>, bool> step(std::shared_ptr<Action> decision_rule);
        std::shared_ptr<Space> getActionSpaceAt(const TState &occupancy_state);
        std::shared_ptr<OccupancyStateInterface> nextState(const std::shared_ptr<OccupancyStateInterface> &, const std::shared_ptr<JointDeterministic> &, number, std::shared_ptr<HSVI>, bool) const;
        TState nextState(const TState &, const std::shared_ptr<Action> &, number = 0, std::shared_ptr<HSVI> = nullptr) const;
        double getReward(const TState &occupancy_state, const std::shared_ptr<Action> &decision_rule) const;
    };
} // namespace sdm
#include <sdm/world/occupancy_mdp.tpp>
