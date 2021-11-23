#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>

namespace sdm
{
    class ActionSelectionMaxplanLP : public MaxPlanSelectionBase, public DecentralizedLP
    {
    public:
        ActionSelectionMaxplanLP();
        ActionSelectionMaxplanLP(const std::shared_ptr<SolvableByDP> &world);


        /**
         * @brief Create a Objective Constraint of the LP
         * 
         * @param occupancy_state 
         * @param var 
         * @param obj 
         * @param t 
         */
        void createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

    protected:
        
        Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<BeliefInterface> &hyperplane, number t);

        std::shared_ptr<BeliefInterface> current_hyperplane; 
    };
}

#endif