#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
#include <sdm/utils/value_function/action_vf/action_sawtooth_lp.hpp>

namespace sdm
{
    class ActionVFSawtoothLPRelaxed : public ActionVFSawtoothLP
    {
    public:        
        ActionVFSawtoothLPRelaxed();
        ActionVFSawtoothLPRelaxed(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        Pair<std::shared_ptr<Action>, double> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

        /**
         * @brief Create the variable which will be used to resolve the LP
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloEnv & : env 
         * @param IloNumVarArray & : var 
         * @param double& index
         * @param number t : Time Step 
         */
        void createVariables(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);
        
        /**
         * @brief Create the constraints of the LP
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloEnv & : env 
         * @param IloModel & : var 
         * @param IloRangeArray& : model 
         * @param IloRangeArray &con
         * @param IloNumVarArray &var
         * @param double& index
         * @param number t : Time Step 
         */
        void createConstraints(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Create a Objective Constraint of the LP with a specialisation for the Occupancy State
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloModel & : var 
         * @param IloObjective& : obj 
         * @param double& index
         * @param number t : Time Step 
         */
        void createObjectiveFunctionOccupancy(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

    protected :         
        
        /**
         * @brief Create the constraints with the Big M formalim specialized for the case Occupancy State
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloEnv & : env 
         * @param IloModel & : var 
         * @param IloRangeArray& : model 
         * @param IloRangeArray &con
         * @param IloNumVarArray &var
         * @param double& index
         * @param number t : Time Step 
         */
        void createSawtoothBigMOccupancy(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        
        
        /**
         * @brief Create the constraints with IloIfThen formalim specialized for the case Serial Occupancy State
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloEnv & : env 
         * @param IloModel & : var 
         * @param IloRangeArray& : model 
         * @param IloRangeArray &con
         * @param IloNumVarArray &var
         * @param double& index
         * @param number t : Time Step 
         */
        void createSawtoothIloIfThenOccupancy(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var,number t);
    };
}
