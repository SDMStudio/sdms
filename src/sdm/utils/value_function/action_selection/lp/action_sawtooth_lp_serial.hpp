// #ifdef WITH_CPLEX
// #pragma once

// #include <sdm/utils/value_function/action_selection/lp/action_sawtooth_lp.hpp>

// namespace sdm
// {
//     class ActionSelectionSawtoothLPSerial : public ActionSelectionSawtoothLP
//     {
//     public:
//         ActionSelectionSawtoothLPSerial();
        
//         ActionSelectionSawtoothLPSerial(const std::shared_ptr<SolvableByDP> &world, TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program);

//         void createVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);

//         void createConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

//         std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t);

//         /**
//          * @brief Get the underlying serial MPOMDP problem
//          */
//         std::shared_ptr<SerialMPOMDPInterface> getSerialMPOMDP() const;

//     protected:
//         std::shared_ptr<SerialMPOMDPInterface> serial_mpomdp;


//         /**
//          * @brief Create constraints with the Big M formalim
//          * 
//          * @param const std::shared_ptr<ValueFunctionInterface>& vf : Value function
//          * @param const std::shared_ptr<State> & occupancy_state : current state
//          * @param const std::shared_ptr<JointHistoryInterface>& joint_history
//          * @param const std::shared_ptr<State> &next_hidden_state
//          * @param const std::shared_ptr<Observation> &next_observation
//          * @param const std::shared_ptr<JointHistoryInterface> &next_joint_history
//          * @param const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state 
//          * @param double probability
//          * @param double difference
//          * @param IloEnv & : env 
//          * @param IloRangeArray &con
//          * @param IloModel & : var 
//          * @param double& index
//          * @param number t : Time Step 
//          */
//         void createSawtoothBigM(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

//         /**
//          * @brief Create the constraints with IloIfThen formalim specialized
//          * 
//          * @param const std::shared_ptr<ValueFunction>& vf : Value function
//          * @param const std::shared_ptr<State> & occupancy_state : current state
//          * @param IloEnv & : env 
//          * @param IloModel & : var 
//          * @param IloRangeArray& : model 
//          * @param IloRangeArray &con
//          * @param IloNumVarArray &var
//          * @param double& index
//          * @param number t : Time Step 
//          */
//         void createSawtoothIloIfThen(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t);

//         /**
//          * @brief Get the 
//          * 
//          * @param state 
//          * @param t 
//          * @return std::shared_ptr<Joint<std::shared_ptr<Observation>>> 
//          */
//         std::shared_ptr<Joint<std::shared_ptr<Observation>>> determineNextJointObservation(const std::shared_ptr<JointHistoryInterface> &joint_history, number t);

//         void createInitialConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
//     };
// }

// #endif