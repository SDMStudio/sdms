#include <sdm/utils/linear_programming/lp_problem.hpp>
#include <sdm/utils/value_function/backup/variable_naming.hpp>

namespace sdm
{
    class DecentralizedLP : public VarNaming, public LPBase
    {
    public :

        /**
         * @brief Set decentralized variables 
         * 
         * @param const std::shared_ptr<State>& : current state
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number& : index variable
         * @param number : time step
         */
        virtual void setDecentralizedVariables(const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t) = 0;

        /**
         * @brief Set decentralized constraints 
         * @param const std::shared_ptr<State>& 
         * @param IloEnv& 
         * @param IloRangeArray&
         * @param IloNumVarArray&
         * @param number&
         * @param number : time step
         */
        virtual void setDecentralizedConstraints(const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t) = 0;

    };
}