#ifdef WITH_CPLEX
#pragma once

#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>
#include <sdm/utils/value_function/action_selection/action_selection_base.hpp>
#include <sdm/utils/value_function/vfunction/sawtooth_value_function.hpp>

namespace sdm
{

    class ActionSelectionSawtoothLP : public ActionSelectionBase, public DecentralizedLP
    {
    public:
        ActionSelectionSawtoothLP();
        ActionSelectionSawtoothLP(const std::shared_ptr<SolvableByDP> &world, Config config);
        ActionSelectionSawtoothLP(const std::shared_ptr<SolvableByDP> &world, TypeOfResolution current_type_of_resolution, number bigM_value, TypeSawtoothLinearProgram type_of_linear_program);

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
        virtual void createObjectiveFunction(const std::shared_ptr<ValueFunctionInterface> &vf, const std::shared_ptr<State> &occupancy_state, IloNumVarArray &var, IloObjective &obj, number t);

        std::shared_ptr<SawtoothValueFunction> getSawtoothValueFunction() const;

    protected:
        /**
         * @brief The type of resolution.
         */
        TypeOfResolution current_type_of_resolution_;

        /**
         * @brief The type of linear program.
         */
        TypeSawtoothLinearProgram type_of_linear_program_ = PLAIN_SAWTOOTH_LINER_PROGRAMMING;

        /**
         * @brief Big M hyperparameter.
         *
         * See CPLEX documentation for more information about how to use it.
         *
         */
        number bigM_value_;

        /**
         * @brief The sawtooth value function
         */
        std::shared_ptr<SawtoothValueFunction> sawtooth_vf;

        /**
         * @brief Create constraints based on BigM method.
         *
         * @param value_function the sawtooth value function
         * @param occupancy_state the current one step uncompressed occupancy state
         * @param next_occupancy_state the next one step occupancy state
         * @param next_state the next state
         * @param next_joint_history the next history
         * @param next_observation the next observation
         * @param env CPLEX environment
         * @param con CPLEX constraints
         * @param var CPLEX variables
         * @param index index of the constraint
         * @param t the time step
         */
        virtual void createSawtoothBigM(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<OccupancyStateInterface> &next_occupancy_state, const std::shared_ptr<State> &next_state, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<Observation> &next_observation, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        /**
         * @brief Create constraints based on IloIfThen method
         *
         * @param value_function the sawtooth value function
         * @param occupancy_state the current one step uncompressed occupancy state
         * @param next_occupancy_state the next one step occupancy state
         * @param next_state the next state
         * @param next_joint_history the next history
         * @param next_observation the next observation
         * @param env CPLEX environment
         * @param model CPLEX model
         * @param var CPLEX variables
         * @param t the time step
         */
        virtual void createSawtoothIloIfThen(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, const std::shared_ptr<OccupancyStateInterface> &next_occupancy_state, const std::shared_ptr<State> &next_state, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<Observation> &next_observation, IloEnv &env, IloModel &model, IloNumVarArray &var, number t);

        virtual void createInitialConstraints(const std::shared_ptr<OccupancyStateInterface> &occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        virtual void createOmegaConstraints(IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

        virtual std::shared_ptr<Joint<std::shared_ptr<Observation>>> determineNextJointObservation(const std::shared_ptr<JointHistoryInterface> &, number t);

        double computeRatio(const std::shared_ptr<BeliefMDPInterface> &oMDP,
                            const std::shared_ptr<BeliefInterface> &belief, const std::shared_ptr<JointHistoryInterface> joint_history,
                            const std::shared_ptr<Action> &action, const std::shared_ptr<OccupancyStateInterface> &s_k,
                            const std::shared_ptr<State> &next_state, const std::shared_ptr<JointHistoryInterface> &next_joint_history,
                            const std::shared_ptr<Observation> &observation, number t);
    };
}

#endif