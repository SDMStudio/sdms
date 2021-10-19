#pragma once

#include <sdm/utils/value_function/action_selection/constraint_programming_selection.hpp>
#include <sdm/utils/linear_programming/variable_naming.hpp>

// #include "../../toulbar2/src/toulbar2lib.hpp"
#include "toulbar2lib.hpp"

namespace sdm
{
    class ActionSelectionMaxplanWCSP : public ConstraintProgrammingSelection, public VarNaming
    {
    public:
        using TData = std::shared_ptr<State>;

        ActionSelectionMaxplanWCSP();
        ActionSelectionMaxplanWCSP(const std::shared_ptr<SolvableByHSVI> &world);

        Pair<std::shared_ptr<Action>, double> createAndSolveCP(const std::shared_ptr<ValueFunction> &, const std::shared_ptr<State> &state, number t);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t);

        // Fonction temporaire le temps de bien comprendre
        Pair<std::shared_ptr<Action>, double> createWCSPProblem(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t);

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

        double getValueAt(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_hyperplan, number t);

        std::shared_ptr<BeliefInterface> tmp_representation;

        void determineMaxValue(const std::shared_ptr<State> &state, number t);
    };
}
