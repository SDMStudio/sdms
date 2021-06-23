#pragma once

#include <sdm/core/state/occupancy_state_interface.hpp>
#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
// #include <sdm/types.hpp>
// #include <sdm/core/joint.hpp>
// #include <sdm/core/space/discrete_space.hpp>
// #include <sdm/core/state/state.hpp>
// #include <sdm/core/state/beliefs.hpp>
// #include <sdm/core/state/occupancy_state.hpp>

// #include <sdm/world/solvable_by_hsvi.hpp>
// #include <sdm/world/discrete_mdp.hpp>
// #include <sdm/world/discrete_pomdp.hpp>
// #include <sdm/world/discrete_decpomdp.hpp>
// #include <sdm/world/belief_mdp.hpp>
// #include <sdm/world/base/base_occupancy_mdp.hpp>

// #include <sdm/utils/linear_algebra/vector.hpp>
// #include <sdm/core/action/joint_det_decision_rule.hpp>

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
        OccupancyMDP(std::shared_ptr<MPOMDPInterface> dpomdp, number max_history_length = -1);

        void initialize(number history_length);
        std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> decision_rule);
        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &occupancy_state);
        std::shared_ptr<State> nextState(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number = 0, std::shared_ptr<HSVI> = nullptr) const;
        double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule) const;

    protected : 
        std::shared_ptr<MPOMDPInterface> getUnderlyingMPOMDP() const;
        std::shared_ptr<OccupancyStateInterface> nextState(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number, std::shared_ptr<HSVI>, bool) const;
    };
} // namespace sdm
