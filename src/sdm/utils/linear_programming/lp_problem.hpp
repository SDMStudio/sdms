#pragma once

#include <sdm/utils/linear_programming/lp_problem_interface.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    class LPBase : public LPInterface
    {
    public:
        LPBase();
        LPBase(const std::shared_ptr<SolvableByHSVI>&);
        ~LPBase();

        /**
         * @brief Main function who is used to create the Linear program
         * 
         * @param occupancy_state 
         * @param t 
         * @return Pair<std::shared_ptr<Action>,double> 
         */
        Pair<std::shared_ptr<Action>,double> createLP(const std::shared_ptr<State> &occupancy_state, number t);

        /**
         * @brief Create the variable which will be used to resolve the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param var 
         * @param t 
         */
        virtual void createVariables(const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t) = 0;

        /**
         * @brief Create a Objective Constraint of the LP
         * 
         * @param occupancy_state 
         * @param var 
         * @param obj 
         * @param t 
         */
        virtual void createObjectiveConstrains(const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t) = 0;

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
        virtual void createConstraints(const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t) = 0;

        /**
         * @brief Get the result of the variable created
         * 
         * @param occupancy_state 
         * @param cplex 
         * @param var 
         * @param t 
         * @return std::shared_ptr<Action> 
         */
        virtual std::shared_ptr<Action> getVariableResult(const std::shared_ptr<State> &occupancy_state,const IloCplex &cplex, const IloNumVarArray &var, number t) =0;

    protected : 
        std::shared_ptr<SolvableByHSVI> world_;

        // /**
        //  * @brief The temporary one-stage value function represention.
        //  */
        // std::shared_ptr<VectorInterface<std::shared_ptr<State>, double>> tmp_representation;
    };
}