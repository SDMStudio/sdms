#pragma once

#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>
#include <sdm/utils/linear_programming/variable_naming.hpp>

// #include "../../toulbar2/src/toulbar2lib.hpp"
#include "toulbar2lib.hpp"

namespace sdm
{
    class ActionSelectionMaxplanWCSP : public MaxPlanSelectionBase, public VarNaming
    {
    public:
        using TData = std::shared_ptr<State>;

        ActionSelectionMaxplanWCSP();
        ActionSelectionMaxplanWCSP(const std::shared_ptr<SolvableByDP> &world);

        /**
         * @brief Compute the greedy action and corresponding value for a specific next hyperplan (saved in the temporary representation).
         *
         * @param vf the value function
         * @param state the state
         * @param t the time step
         * @return action and corresponding value
         */
        Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, number t);

        Pair<std::shared_ptr<Action>, double> createAndSolveWCSP(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<State> &state, number t);

        double getWeight(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface> occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history, const std::shared_ptr<Action> action, number t);

        // Fonction temporaire le temps de bien comprendre
        // Pair<std::shared_ptr<Action>, double> createWCSPProblem(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t);

    protected:
        /**
         * @brief defines the maximum value in the domain of the payoff function
         * 
         */
        double max;

        /**
         * @brief factor used to convert real values into integer costs
         * 
         */
        long offset = 1000000000000;

        /**
         * @brief Returns a cost value
         * 
         * @param double defines a real value
         */
        long getCost(double);

        void determineMaxValue(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);
    };
}
