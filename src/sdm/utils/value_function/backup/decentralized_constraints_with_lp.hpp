#pragma once
#include <ilcplex/ilocplex.h>
// #include <sdm/utils/linear_algebra/vector.hpp>
// #include <sdm/utils/value_function/variable_naming.hpp>
// #include <sdm/core/states.hpp>
// #include <sdm/exception.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/core/states.hpp>

namespace sdm
{
    class DecentralizedConstraintsLP  //: public VarNaming<TVector, TAction, TValue>
    {
    public:
        DecentralizedConstraintsLP(const std::shared_ptr<SolvableByHSVI>&);

        /**
         * @brief Get the decentralized decision rule from the result
         * 
         * @param const IloCplex&
         * @param const IloNumVarArray&
         * @param const std::shared_ptr<OccupancyStateInterface>& : current state
         * @param number : time step
         * @return TAction 
         */
        std::shared_ptr<Action> getDecentralizedVariables(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);

        /**
         * @brief Set decentralized variables 
         * 
         * @param const std::shared_ptr<OccupancyStateInterface>& : current state
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number& : index variable
         * @param number : time step
         */
        void setDecentralizedVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Set decentralized constraints 
         * @param const std::shared_ptr<OccupancyStateInterface>& 
         * @param IloEnv& 
         * @param IloRangeArray&
         * @param IloNumVarArray&
         * @param number&
         * @param number : time step
         */
        void setDecentralizedConstraints(const std::shared_ptr<OccupancyStateInterface>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        // /**
        //  * @brief Set decentralized constraints 
        //  * @param const TVector& 
        //  * @param IloEnv& 
        //  * @param IloRangeArray&
        //  * @param IloNumVarArray&
        //  * @param number&
        //  * @param number : time step
        //  */
        // template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int> = 0>
        // void setDecentralizedConstraints(const TVector&, IloEnv&, IloRangeArray&, IloNumVarArray&, number&, number);


        // /**
        //  * @brief Set decentralized constraints 
        //  * @param const TVector& 
        //  * @param IloEnv& 
        //  * @param IloRangeArray&
        //  * @param IloNumVarArray&
        //  * @param number&
        //  * @param number : time step
        //  */
        // template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        // void setDecentralizedConstraints(const TVector&, IloEnv&, IloRangeArray&, IloNumVarArray&, number&, number);

    protected : 
        std::shared_ptr<SolvableByHSVI> world_;

        std::shared_ptr<Action> getDecentralizedVariablesOccupancy(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);
        std::shared_ptr<Action> getDecentralizedVariablesSerialOccupancy(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);

        void setDecentralizedVariablesOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);
        void setDecentralizedVariablesSerialOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        void setDecentralizedConstraintsOccupancy(const std::shared_ptr<OccupancyStateInterface>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        void setDecentralizedConstraintsSerialOccupancy(const std::shared_ptr<OccupancyStateInterface>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);


    };
}