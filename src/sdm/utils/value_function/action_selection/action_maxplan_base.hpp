
#pragma once

#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>

namespace sdm
{

    class MaxPlanSelectionBase : public ActionSelectionBase
    {
    public:
        MaxPlanSelectionBase();
        MaxPlanSelectionBase(const std::shared_ptr<SolvableByDP> &world);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         *
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t);

    protected:
        /**
         * @brief Compute the greedy action and corresponding value for a specific next hyperplan (saved in the temporary representation).
         *
         * @param vf the value function
         * @param state the state
         * @param t the time step
         * @return action and corresponding value
         */
        virtual Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &hyperplane, number t) = 0;

        double getWeight(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface>& occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, const std::shared_ptr<BeliefInterface> &hyperplane, number t);
    };
}
