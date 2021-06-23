
#pragma once

#include <sdm/utils/value_function/backup/sawtooth_backup.hpp>
#include <sdm/utils/value_function/backup/decentralized_constraints_with_lp.hpp>
namespace sdm
{
    class SawtoothLPBackup : public SawtoothBackup, DecentralizedConstraintsLP
    {
    public:
        using TData = double;

        SawtoothLPBackup();
        SawtoothLPBackup(const std::shared_ptr<SolvableByHSVI>& ,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution);

        TData backup(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        // std::pair<double, std::shared_ptr<State>> getMaxAt(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        std::shared_ptr<Action> getBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

        Pair<std::shared_ptr<Action>,double> getGreedy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);
        void setGreedyVariables(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t);
        void setGreedyObjective(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

        double getQValueRelaxation(const  std::shared_ptr<OccupancyStateInterface> &compressed_occupancy_state, const std::shared_ptr<JointHistoryInterface> joint_history, std::shared_ptr<Action> action, number t);

        void setGreedySawtooth(const std::shared_ptr<OccupancyStateInterface> &compressed_occupancy_state, IloModel &model, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

    protected : 

        /**
         * @brief The type of resolution.
         */
        TypeOfResolution current_type_of_resolution_;

        /**
         * @brief The type of linear program.
         */
        TypeSawtoothLinearProgram csawtooth_lp_ = PLAIN_SAWTOOTH_LINER_PROGRAMMING;

        number bigM_value_;

        /**
         * @brief The function getGreedy with the specialisation of Full Sawtooth (TypeSawtoothLinearProgram::PLAIN_SAWTOOTH_LINER_PROGRAMMING)
         * 
         */
        Pair<std::shared_ptr<Action>,double> greedyFullSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface>  &occupancy_state, number t);
        
        /**
         * @brief The function getGreedy with the specialisation of Relaxed Sawtooth (TypeSawtoothLinearProgram::RELAXED_SAWTOOTH_LINER_PROGRAMMING)
         * 
         */
        Pair<std::shared_ptr<Action>,double> greedyRelaxedSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface> &occupancy_state, number t);

        Pair<std::shared_ptr<Action>,double> greedy(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<OccupancyStateInterface> &hyperplan, number t);

        void setGreedyObjectiveOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t)
        void setGreedyObjectiveSerialOccupancy(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t)

    };
}