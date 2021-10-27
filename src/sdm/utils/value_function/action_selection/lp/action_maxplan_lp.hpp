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

        double getWeight(const std::shared_ptr<ValueFunctionInterface> &value_function, const std::shared_ptr<OccupancyStateInterface> occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history, const std::shared_ptr<Action> action, number t);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        // Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface>& vf, const std::shared_ptr<State>& state, number t);

        /**
         * @brief Create the variable which will be used to resolve the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param var 
         * @param t 
         */
        void createVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Create a Objective Constraint of the LP
         * 
         * @param occupancy_state 
         * @param var 
         * @param obj 
         * @param t 
         */
        void createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

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
        void createConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Set decentralized variables 
         * 
         * @param const std::shared_ptr<State>& : current state
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number& : index variable
         * @param number : time step
         */
        virtual void createDecentralizedVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Set decentralized constraints 
         * @param const std::shared_ptr<State>& 
         * @param IloEnv& 
         * @param IloRangeArray&
         * @param IloNumVarArray&
         * @param number&
         * @param number : time step
         */
        virtual void createDecentralizedConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Get the result of the variable created
         * 
         * @param state 
         * @param cplex 
         * @param var 
         * @param t 
         * @return std::shared_ptr<Action> 
         */
        virtual std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t);

        /**
         * @brief Get the world
         * 
         * @return the world 
         */
        std::shared_ptr<SolvableByDP> getWorld() const;
    };
}

#endif