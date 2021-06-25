#pragma once

#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>

namespace sdm
{
    class ActionVFSawtoothLPOccupancy : public ActionVFSawtoothLP
    {
    public:
        ActionVFSawtoothLPOccupancy();
        ActionVFSawtoothLPOccupancy(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution);

    protected : 
        void createInitialConstrainte(const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        void createSawtoothBigM(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        void createSawtoothIloIfThen(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var,number t);

        /**
         * @brief Set decentralized variables 
         * 
         * @param const std::shared_ptr<State>& : current state
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number& : index variable
         * @param number : time step
         */
        void createDecentralizedVariables(const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Set decentralized constraints 
         * @param const std::shared_ptr<State>& 
         * @param IloEnv& 
         * @param IloRangeArray&
         * @param IloNumVarArray&
         * @param number&
         * @param number : time step
         */
        void createDecentralizedConstraints(const std::shared_ptr<State>& state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Get the result of the variable created
         * 
         * @param state 
         * @param cplex 
         * @param var 
         * @param t 
         * @return std::shared_ptr<Action> 
         */
        std::shared_ptr<Action> getVariableResult(const std::shared_ptr<State> &state,const IloCplex &cplex, const IloNumVarArray &var, number t);
    };
}
