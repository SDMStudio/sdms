#pragma once
#include <ilcplex/ilocplex.h>
#include <sdm/utils/value_function/backup/variable_naming.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/core/states.hpp>
#include <sdm/utils/linear_algebra/vector_interface.hpp>

namespace sdm
{
    class DecentralizedConstraintsLP : public VarNaming
    {
    public:
        DecentralizedConstraintsLP();
        DecentralizedConstraintsLP(const std::shared_ptr<SolvableByHSVI>&);

        virtual Pair<std::shared_ptr<Action>,double> getGreedy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t) = 0;
        virtual void setGreedyVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t) = 0;
        virtual void setGreedyObjective(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t) = 0;

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

    protected : 
        std::shared_ptr<SolvableByHSVI> world_;

        /**
         * @brief The temporary one-stage value function represention.
         */
        std::shared_ptr<VectorInterface<std::shared_ptr<State>, double>> tmp_representation;

        std::shared_ptr<Action> getDecentralizedVariablesOccupancy(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);
        std::shared_ptr<Action> getDecentralizedVariablesSerialOccupancy(const IloCplex &cplex, const IloNumVarArray &var, const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);

        void setDecentralizedVariablesOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);
        void setDecentralizedVariablesSerialOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        void setDecentralizedConstraintsOccupancy(const std::shared_ptr<OccupancyStateInterface>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        void setDecentralizedConstraintsSerialOccupancy(const std::shared_ptr<OccupancyStateInterface>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);


    };
}