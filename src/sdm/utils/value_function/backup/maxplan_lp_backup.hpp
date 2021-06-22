#pragma once

#include <sdm/utils/value_function/backup/maxplan_backup.hpp>
#include <sdm/utils/value_function/backup/decentralized_constraints_with_lp.hpp>

namespace sdm
{
    class MaxPlanLPBackup : public MaxPlanBackup, DecentralizedConstraintsLP
    {
    public:
        using TData = std::shared_ptr<State>;

        MaxPlanLPBackup();
        MaxPlanLPBackup(const std::shared_ptr<SolvableByHSVI>& );

        virtual TData backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        virtual std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

    protected : 
        Pair<std::shared_ptr<State>,std::shared_ptr<Action>> getBestActionAndMaxHyperplan(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        
        Pair<std::shared_ptr<Action>,double> greedyMaxPlane(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<OccupancyStateInterface> &hyperplan, number t);

        void setGreedyVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t);

        void setGreedyObjective(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, const std::shared_ptr<OccupancyStateInterface> &hyperplan, number t);

    };
}