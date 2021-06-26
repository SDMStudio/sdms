#pragma once

#include <sdm/world/belief_mdp.hpp>
#include <sdm/world/base/mpomdp_interface.hpp>
#include <sdm/core/state/interface/history_interface.hpp>
#include <sdm/core/state/interface/occupancy_state_interface.hpp>

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
        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &occupancy_state, number t = 0);
        std::shared_ptr<State> nextState(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr) const;
        double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &decision_rule, number t = 0) const;
        double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t) const;

    protected:
        std::shared_ptr<MPOMDPInterface> getUnderlyingMPOMDP() const;
        std::shared_ptr<OccupancyStateInterface> nextState(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number, const std::shared_ptr<HSVI> &, bool) const;
        std::shared_ptr<Action> applyDecisionRule(const std::shared_ptr<OccupancyStateInterface> &ostate, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &decision_rule, number t) const;

        std::shared_ptr<HistoryInterface> initial_history_, current_history_;
    };
} // namespace sdm
