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
        using TData = double;

        ActionSelectionSawtoothLP();
        ActionSelectionSawtoothLP(const std::shared_ptr<SolvableByHSVI>& world,TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program);

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
        virtual void createVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number &index, number t);

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
        virtual void createConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

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
        void createObjectiveFunction(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);
        Pair<std::shared_ptr<State>, double> evaluate(const std::shared_ptr<ValueFunction> &vf,const std::shared_ptr<State> &state_tmp,const std::shared_ptr<Action>& decision_rule, number t);

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
        virtual void createSawtoothBigM(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

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
        // virtual void createSawtoothIloIfThen(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t);

        virtual void createSawtoothIloIfThen(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var, number t);

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
        virtual double getSawtoothMinimumRatio(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, double denominator, number t);

        /**
         * @brief Calculate the Q Value Relaxation i.e. \sum_{o} a(u|o) \sum_{x} s(x,o) * Q_MDP(x,u)

         * 
         * @param const std::shared_ptr<ValueFunction>& vf : Value function
         * @param const std::shared_ptr<State> & occupancy_state : current state
         * @param const std::shared_ptr<JointHistoryInterface> joint_history
         * @param std::shared_ptr<Action> action
         * @param number t : Time Step 
         */
        virtual double getQValueRelaxation(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<Action> &action, number t);

        /**
         * @brief Get the 
         * 
         * @param state 
         * @param t 
         * @return std::shared_ptr<Joint<std::shared_ptr<Observation>>> 
         */
        virtual std::shared_ptr<Joint<std::shared_ptr<Observation>>> determineNextJointObservation(const std::shared_ptr<JointHistoryInterface> &, number t);

        // std::shared_ptr<MappedVector<std::shared_ptr<State>, double>> representation;

        // std::unordered_map<std::shared_ptr<State>, std::shared_ptr<Action>> state_linked_to_decision_rule;

        virtual void createDecentralizedVariables(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);
        virtual void createDecentralizedConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        virtual std::shared_ptr<Action> getVariableResult(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const IloCplex &cplex, const IloNumVarArray &var, number t);

        virtual void createInitialConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
        void createOmegaConstraints(const Pair<std::shared_ptr<State>,double> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index);
        // void createUpperBoundLimitConstraints(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,  IloEnv &env,IloRangeArray &con, IloNumVarArray &var,number &index, number t);

        Pair<std::shared_ptr<Action>, double> selectBestActionFull(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t);
        Pair<std::shared_ptr<Action>, double> selectBestActionRelaxed(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t);
        Pair<std::shared_ptr<Action>, double> selectBestActionRelaxedV2(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, number t);

        /**
         * @brief Create all the Omega Variable i.e. \forall k, (o',x') \omega_k(x',o')
         * 
         * @param env 
         * @param var 
         * @param index 
         */
        void createOmegaVariable(IloEnv &env, IloNumVarArray &var, number &index);

        /**
         * @brief Create the objective variable 
         * 
         * @param vf 
         * @param state 
         * @param env 
         * @param var 
         * @param index 
         * @param t 
         */
        void createObjectiveVariable(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, IloEnv &env, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Compute the difference v_kappa - V_kappa i.e. the value of point at t+1 minus the relaxation value of the point at t+1
         * 
         * @param vf 
         * @param state_AND_value 
         * @param t 
         * @return double 
         */
        double computeDifference(const std::shared_ptr<ValueFunction> &vf,const Pair<std::shared_ptr<State>,double>& state_AND_value, number t);


        void createConstraintsKnowingInformation(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface> &, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double denominator , double difference , IloEnv &env, IloModel &model, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        double computeSawtooth( const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state, const std::shared_ptr<Action>& action, const std::shared_ptr<JointHistoryInterface> &joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, double denominator , double difference, number t);

        void createGlobalConstraint(const std::shared_ptr<ValueFunction> &vf, const std::shared_ptr<State> &state,IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);
    };
}

#endif