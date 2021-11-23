#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/linear_programming/lp_problem.hpp>

namespace sdm
{
    class IndividualLP : public LPBase
    {
    public :

        IndividualLP();
        IndividualLP(const std::shared_ptr<SolvableByDP>&);

        /**
         * @brief Create the variable which will be used to resolve the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param var 
         * @param t 
         */
        void createVariables(const std::shared_ptr<ValueFunctionInterface>&vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var,number &index, number t);
        
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
        void createConstraints(const std::shared_ptr<ValueFunctionInterface>&vf, const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Get the result of the variable created
         * 
         * @param occupancy_state 
         * @param cplex 
         * @param var 
         * @param t 
         * @return std::shared_ptr<Action> 
         */
        std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunctionInterface>&vf, const std::shared_ptr<State> &occupancy_state,const IloCplex &cplex, const IloNumVarArray &var, number t);

    };
}

#endif