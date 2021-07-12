#pragma once

#include <sdm/core/action/action.hpp>
#include <ilcplex/ilocplex.h>
#include <sdm/core/state/state.hpp>
#include <sdm/utils/value_function/value_function.hpp>


namespace sdm
{
    class LPInterface
    {
    public : 

        /**
         * @brief Main function who is used to create the Linear program
         * 
         * @param occupancy_state 
         * @param t 
         * @return Pair<std::shared_ptr<Action>,double> 
         */
        virtual Pair<std::shared_ptr<Action>,double> createLP(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, number t) = 0;

        /**
         * @brief Create the variable which will be used to resolve the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param var 
         * @param t 
         */
        virtual void createVariables(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var,number &index, number t) = 0;
        
        /**
         * @brief Create a Objective Constraint of the LP
         * 
         * @param occupancy_state 
         * @param var 
         * @param obj 
         * @param t 
         */
        virtual void createObjectiveFunction(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t) = 0;
        
        /**
         * @brief Create the constraints of the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param con 
         * @param var 
         * @param index 
         * @param t 
         */
        virtual void createConstraints(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t) = 0;

        /**
         * @brief Get the result of the variable created
         * 
         * @param occupancy_state 
         * @param cplex 
         * @param var 
         * @param t 
         * @return std::shared_ptr<Action> 
         */
        virtual std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state,const IloCplex &cplex, const IloNumVarArray &var, number t) =0;
    };
}