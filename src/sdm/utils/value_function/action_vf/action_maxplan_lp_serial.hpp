#pragma once

#include <sdm/utils/value_function/action_vf/action_maxplan_lp.hpp>

namespace sdm
{
    class ActionVFMaxplanLPSerial : public ActionVFMaxplanLP
    {
    public:
        using TData = std::shared_ptr<State>;
        
        ActionVFMaxplanLPSerial();
        ActionVFMaxplanLPSerial(const std::shared_ptr<SolvableByHSVI>& world);
        
        /**
         * @brief Create a Objective Constraint of the LP
         * 
         * @param occupancy_state 
         * @param var 
         * @param obj 
         * @param t 
         */
        void createObjectiveFunction(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

        void createDecentralizedVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);
        void createDecentralizedConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t);
    };
}
