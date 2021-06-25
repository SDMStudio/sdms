#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

namespace sdm
{
    class DecentralizedLPOccupancy : public DecentralizedLP
    {
    public :

        DecentralizedLPOccupancy();
        DecentralizedLPOccupancy(const std::shared_ptr<SolvableByHSVI>&);

        /**
         * @brief Set decentralized variables 
         * 
         * @param const std::shared_ptr<State>& : current state
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number& : index variable
         * @param number : time step
         */
        void createDecentralizedVariables(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Set decentralized constraints 
         * @param const std::shared_ptr<State>& 
         * @param IloEnv& 
         * @param IloRangeArray&
         * @param IloNumVarArray&
         * @param number&
         * @param number : time step
         */
        void createDecentralizedConstraints(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Get the result of the variable created
         * 
         * @param state 
         * @param cplex 
         * @param var 
         * @param t 
         * @return std::shared_ptr<Action> 
         */
        std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunction>&, const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t);
    };
}