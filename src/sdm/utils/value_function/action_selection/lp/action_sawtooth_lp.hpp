#ifdef WITH_CPLEX

#pragma once

#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

#include <sdm/utils/linear_algebra/mapped_vector.hpp>
namespace sdm
{
    class ActionSelectionSawtoothLP : public ActionSelectionBase, public DecentralizedLP
    {
    public:
        ActionSelectionSawtoothLP();
        ActionSelectionSawtoothLP(const std::shared_ptr<SolvableByDP>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program);

        /**
         * @brief Select the best action and the hyperplan at t+1 associated for a state at a precise time
         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State>& state : current state
         * @param number t : time step
         * @return  Pair<std::shared_ptr<Action>,TData> : best action and the hyperplan at t+1 associated
         */
        virtual Pair<std::shared_ptr<Action>, double> getGreedyActionAndValue(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, number t);

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
        virtual void createVariables(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);

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
        virtual void createConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Create a Objective Constraint of the LP
         * 
         * @param const std::shared_ptr<ValueFunctionInterface>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloModel & : var 
         * @param IloObjective& : obj 
         * @param double& index
         * @param number t : Time Step 
         */
        void createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);
        

    protected:
        /**
         * @brief The type of resolution.
         */
        TypeOfResolution current_type_of_resolution_;

        /**
         * @brief The type of linear program.
         */
        TypeSawtoothLinearProgram type_of_linear_program_ = PLAIN_SAWTOOTH_LINER_PROGRAMMING;

        number bigM_value_;

        std::unordered_map<Pair<std::shared_ptr<State>,double>,std::unordered_map<std::shared_ptr<HistoryInterface>,std::vector<std::shared_ptr<State>>>> all_support;

        /**
         * @brief Create constraints with the Big M formalim
         * 
         * @param const std::shared_ptr<ValueFunctionInterface>& vf : Value function
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
        virtual void createSawtoothBigM(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Create the constraints with IloIfThen formalim specialized
         * 
         * @param const std::shared_ptr<ValueFunctionInterface>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param IloEnv & : env 
         * @param IloModel & : var 
         * @param IloRangeArray& : model 
         * @param IloRangeArray &con
         * @param IloNumVarArray &var
         * @param double& index
         * @param number t : Time Step 
         */
        // virtual void createSawtoothIloIfThen(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t);

        virtual void createSawtoothIloIfThen(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t);


        virtual void createInitialConstraints(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);


        void createOmegaConstraints(const Pair<std::shared_ptr<State>,double> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index);

    };
}

#endif