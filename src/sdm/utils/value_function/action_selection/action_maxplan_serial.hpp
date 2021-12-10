
#pragma once

#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>
#include <sdm/core/state/private_occupancy_state.hpp>
#include <sdm/world/serial_mpomdp.hpp>

namespace sdm
{
    class ActionSelectionMaxplanSerial : public MaxPlanSelectionBase
    {
    public:
        using TData = std::shared_ptr<State>;

        ActionSelectionMaxplanSerial();
        ActionSelectionMaxplanSerial(const std::shared_ptr<SolvableByDP> &world, Config config = {});

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         *
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &hyperplane, number t);
        Pair<std::shared_ptr<Action>, double> selectBestAction(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyState> &state, const std::shared_ptr<BeliefInterface> &next_hyperplane, const std::shared_ptr<HistoryInterface> &ihistory, number t);
        double evaluateAction(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyState> &private_occupancy_state, const std::shared_ptr<Action> &action, const std::shared_ptr<BeliefInterface> &next_hyperplane, number t);

    protected:
        std::shared_ptr<SerialProblemInterface> getSerialProblem() const;
        std::shared_ptr<SerialProblemInterface> serial_mpomdp;
    };
}
