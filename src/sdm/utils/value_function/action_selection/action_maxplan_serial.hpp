#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>

namespace sdm
{
    class ActionSelectionMaxplanSerial : public MaxPlanSelectionBase
    {
    public:
        using TData = std::shared_ptr<State>;

        ActionSelectionMaxplanSerial();
        ActionSelectionMaxplanSerial(const std::shared_ptr<SolvableByDP> &world);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t);

        /**
         * @brief Select the best action and value associated for a state at a precise time and a precise next hyperplan at t +1
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param const std::shared_ptr<BeliefInterface>&
         * @param number t : time step
         * 
         * @return  Pair<std::shared_ptr<Action>,double> : best action and the value associated
         */
        Pair<std::shared_ptr<Action>, double> selectBestDecisionRuleKnowingNextHyperplan(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hyperplan, number t);
        Pair<std::shared_ptr<Action>, double> selectBestActionKnowingNextHyperplanAndHistory(const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hyperplan, const std::shared_ptr<HistoryInterface> &ihistory, number t);
        double evaluationOfHyperplanKnowingNextHyperplanAndDiscreteAction(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_step_hyperplan, number t);
        double evaluateNextExpectedValueAt(const std::shared_ptr<State> &hyperplan, const std::shared_ptr<HistoryInterface> &joint_history, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t);
    };
}

#endif
