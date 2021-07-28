// #pragma once

// #include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
// #include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>

// namespace sdm
// {
//     class ActionVFSawtoothLPRelaxedV2 : public ActionVFSawtoothLP
//     {
//     public:        
//         ActionVFSawtoothLPRelaxedV2();
//         ActionVFSawtoothLPRelaxedV2(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution,number bigM_value);

//         /**
//          * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
//          * 
//          * @param const std::shared_ptr<ValueFunction>& vf : Value function
//          * @param const std::shared_ptr<State>& state : current state
//          * @param number t : time step
//          * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
//          */
//         Pair<std::shared_ptr<Action>, double> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

//         /**
//          * @brief Create the variable which will be used to resolve the LP
//          * 
//          * @param const std::shared_ptr<ValueFunction>& vf : Value function
//          * @param const std::shared_ptr<State> & occupancy_state : current state
//          * @param IloEnv & : env 
//          * @param IloNumVarArray & : var 
//          * @param double& index
//          * @param number t : Time Step 
//          */
//         void createVariables(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);
        
//         /**
//          * @brief Create the constraints of the LP
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
//         void createConstraints(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

//     protected : 

//         std::shared_ptr<State> support_point_set;
//         bool support_empty;
//     };
// }
