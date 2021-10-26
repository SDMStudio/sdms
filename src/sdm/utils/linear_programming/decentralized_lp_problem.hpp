#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/linear_programming/lp_problem.hpp>
#include <sdm/utils/linear_programming/variable_naming.hpp>

namespace sdm
{
    class DecentralizedLP : public VarNaming, public LPBase
    {
    public :

        DecentralizedLP();
        DecentralizedLP(const std::shared_ptr<SolvableByHSVI>&);

        /**
         * @brief Set decentralized variables 
         * 
         * @param const std::shared_ptr<State>& : current state
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number& : index variable
         * @param number : time step
         */
        virtual void createDecentralizedVariables(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t) = 0;
        void createDecentralizedVariablesSerial(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);
        void createDecentralizedVariablesOccupancy(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Set decentralized constraints 
         * @param const std::shared_ptr<State>& 
         * @param IloEnv& 
         * @param IloRangeArray&
         * @param IloNumVarArray&
         * @param number&
         * @param number : time step
         */
        virtual void createDecentralizedConstraints(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t) = 0;
        void createDecentralizedConstraintsSerial(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        void createDecentralizedConstraintsOccupancy(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Get the result of the variable created
         * 
         * @param state 
         * @param cplex 
         * @param var 
         * @param t 
         * @return std::shared_ptr<Action> 
         */
        virtual std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t) = 0;
        std::shared_ptr<Action> getVariableResultSerial(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t);
        std::shared_ptr<Action> getVariableResultOccupancy(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t);

    protected : 

        /**
         * @brief Create the Individual Decentralized Constraints i.e.  set constraint  \sum_{u_i} a_i(u_i|o_i) = 1
         * 
         * @param const std::shared_ptr<State> & : state :
         * @param IloEnv & : env
         * @param IloRangeArray & : con
         * @param IloNumVarArray & : var
         * @param number & : index variable
         * @param number : timestep 
         * @param number : agent_id
         * 
         */
        void createDecentralizedConstraintsIndividual(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t, number agent_id);

        /**
         * @brief Create the Individual Decentralized Variable i.e. Build variables a_i(u_i|o_i)
         * 
         * @param const std::shared_ptr<State> & : state :
         * @param IloEnv & : env
         * @param IloNumVarArray & : var
         * @param number & : index variable
         * @param number : timestep 
         * @param number : agent_id
         * 
         */
        void createDecentralizedVariablesIndividual(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t, number agent_id);

        /**
         * @brief Create the Joint Decentralized Constraints i.e.  set constraint a(u|o) >= \sum_i a_i(u_i|o_i) + 1 - n
         * 
         * @param const std::shared_ptr<State> & : state :
         * @param IloEnv & : env
         * @param IloRangeArray & : con
         * @param IloNumVarArray & : var
         * @param number & : index variable
         * @param number : timestep 
         * 
         */
        void createDecentralizedConstraintsJoint(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Create the Control Decentralized Constraints i.e.  set constraint a(u|o) <= a_i(u_i|o_i)
         * 
         * @param const std::shared_ptr<State> & : state :
         * @param IloEnv & : env
         * @param IloRangeArray & : con
         * @param IloNumVarArray & : var
         * @param number & : index variable
         * @param number : timestep 
         * 
         */
        void createDecentralizedControlConstraints(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        
        /**
         * @brief Create the Joint Decentralized Variable i.e. Build variables a(u|o)
         * 
         * @param const std::shared_ptr<State> & : state :
         * @param IloEnv & : env
         * @param IloNumVarArray & : var
         * @param number & : index variable
         * @param number : timestep 
         * 
         */
        void createDecentralizedVariablesJoint(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Get the Individual Variable Result i.e. return the result for the variable a(u|o) the first vector refers to the vector of discrete action and the second vector refers to the vector of individual history
         * 
         * @param const std::shared_ptr<State> & : state :
         * @param const IloCplex & : cplex
         * @param IloNumVarArray & : var
         * @param number : timestep 
         * @param number : agent_id 
         * 
         * @return Pair<std::vector<std::shared_ptr<Item>>,std::vector<std::shared_ptr<Item>>> , the first vector refers to the vector of discrete action and the second vector refers to the vector of individual history
         */
        Pair<std::vector<std::shared_ptr<Item>>,std::vector<std::shared_ptr<Item>>> getVariableResultIndividual(const std::shared_ptr<ValueFunctionInterface>&vf,const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t, number agent_id);


    };
}

#endif