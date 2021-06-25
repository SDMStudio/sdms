#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

namespace sdm
{
    class ActionVFMaxplanLP : public ActionVFBase<std::shared_ptr<State>>, public DecentralizedLP
    {
    public:
        using TData = std::shared_ptr<State>;
        
        ActionVFMaxplanLP();
        ActionVFMaxplanLP(const std::shared_ptr<SolvableByHSVI>& world);
        
        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>,TData> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

        /**
         * @brief Create the variable which will be used to resolve the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param var 
         * @param t 
         */
        void createVariables(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t);
        
        /**
         * @brief Create a Objective Constraint of the LP
         * 
         * @param occupancy_state 
         * @param var 
         * @param obj 
         * @param t 
         */
        void createObjectiveFunction(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);
        
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
        void createConstraints(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
    };
}
