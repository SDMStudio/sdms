#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>
#include <sdm/utils/value_function/action_selection/action_maxplan_base.hpp>

namespace sdm
{
    class ActionSelectionMaxplanLP : public MaxPlanSelectionBase, public DecentralizedLP
    {
    public:
        using TData = std::shared_ptr<State>;

        ActionSelectionMaxplanLP();
        ActionSelectionMaxplanLP(const std::shared_ptr<SolvableByDP> &world);

        Pair<std::shared_ptr<Action>, double> computeGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t);


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
        
        double getWeight(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface> occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history, const std::shared_ptr<Action> action, number t);

    };
}

#endif