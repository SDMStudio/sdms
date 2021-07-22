#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

#include <sdm/utils/linear_algebra/mapped_vector.hpp>
namespace sdm
{
    class ActionVFSawtoothLP : public ActionVFBase, public DecentralizedLP
    {
    public:
        using TData = double;
        
        ActionVFSawtoothLP();
        ActionVFSawtoothLP(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_sawtooth_resolution);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        std::shared_ptr<Action> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

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
         * @brief Create a Objective Constraint of the LP
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloModel & : var 
         * @param IloObjective& : obj 
         * @param double& index
         * @param number t : Time Step 
         */
        void createObjectiveFunction(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

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

        /**
         * @brief Create a Objective Constraint of the LP with a specialisation for the Serial Occupancy State
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloModel & : var 
         * @param IloObjective& : obj 
         * @param double& index
         * @param number t : Time Step 
         */
        void createObjectiveFunctionSerial(const std::shared_ptr<ValueFunction>&vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

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
         * @brief Create the Relaxed Version of LP problem
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param number t : Time Step 
         */
        std::shared_ptr<Action> createRelaxedSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);

        /**
         * @brief Create the Full Version of LP problem
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param number t : Time Step 
         */
        std::shared_ptr<Action> createFullSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);

        /**
         * @brief Create constraints with the Big M formalim
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param const std::shared_ptr<JointHistoryInterface>& joint_history
         * @param const std::shared_ptr<State> &next_hidden_state
         * @param const std::shared_ptr<Observation> &next_observation
         * @param const std::shared_ptr<JointHistoryInterface> &next_joint_history
         * @param const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state 
         * @param double probability
         * @param double difference
         * @param IloEnv & : env 
         * @param IloRangeArray &con
         * @param IloModel & : var 
         * @param double& index
         * @param number t : Time Step 
         */
        void createSawtoothBigM(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        
        /**
         * @brief Create the constraints with the Big M formalim specialized for the case Serial Occupancy State
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
        void createSawtoothBigMSerial(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        
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
         * @brief Create the constraints with IloIfThen formalim specialized
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
        void createSawtoothIloIfThen(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var,number t);
        
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
        void createSawtoothIloIfThenSerial(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var,number t);
        
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
       
        /**
         * @brief Calculate the 
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param const std::shared_ptr<JointHistoryInterface>& joint_history
         * @param const std::shared_ptr<Action>& action
         * @param std::shared_ptr<State> next_hidden_state
         * @param const std::shared_ptr<Observation> next_observation 
         * @param double denominator 
         * @param double difference 
         * @param number t : Time Step 
         */
        double getQValueRealistic(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &compressed_occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, std::shared_ptr<State> next_hidden_state, const std::shared_ptr<Observation> next_observation, double denominator, double difference,number t);
        
        /**
         * @brief Calculate the Ratio of Sawtooth
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param const const std::shared_ptr<JointHistoryInterface>& joint_history
         * @param const std::shared_ptr<Action>& action
         * @param const std::shared_ptr<State>& next_hidden_state
         * @param const std::shared_ptr<Observation>& next_observation
         * @param double denominator
         * @param number t : Time Step 
         */
        double getSawtoothMinimumRatio(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, const std::shared_ptr<State>& next_hidden_state, const std::shared_ptr<Observation>& next_observation, double denominator, number t);
        
        /**
         * @brief Calculate the Q Value Relaxation i.e. \sum_{o} a(u|o) \sum_{x} s(x,o) * Q_MDP(x,u)

         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param const std::shared_ptr<JointHistoryInterface> joint_history
         * @param std::shared_ptr<Action> action
         * @param number t : Time Step 
         */
        double getQValueRelaxation(const std::shared_ptr<ValueFunction>&vf,const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, number t);

        std::shared_ptr<Joint<std::shared_ptr<Observation>>> determineNextJointObservation(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>&);
        std::set<std::shared_ptr<JointHistoryInterface>> determineJointHistory(const std::shared_ptr<State> &, const std::shared_ptr<JointHistoryInterface>&, const std::shared_ptr<Observation>&);

        std::shared_ptr<MappedVector<std::shared_ptr<State>,double>> representation;

        std::unordered_map<std::shared_ptr<State>,std::shared_ptr<Action>> state_linked_to_decision_rule;
    };
}
