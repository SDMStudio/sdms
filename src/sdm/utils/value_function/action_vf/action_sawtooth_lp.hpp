#pragma once

#include <sdm/utils/value_function/action_vf/action_vf_base.hpp>
#include <sdm/utils/linear_programming/decentralized_lp_problem.hpp>

namespace sdm
{
    class ActionVFSawtoothLP : public ActionVFBase<double>, public DecentralizedLP
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
        Pair<std::shared_ptr<Action>,TData> selectBestAction(const std::shared_ptr<ValueFunction>& vf, const std::shared_ptr<State>& state, number t);

        /**
         * @brief Create the variable which will be used to resolve the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param var 
         * @param t 
         */
        void createVariables(const std::shared_ptr<State> &occupancy_state, IloEnv &env, IloNumVarArray &var, number t);
        
        /**
         * @brief Create the constraints of the LP
         * 
         * @param occupancy_state 
         * @param env 
         * @param con 
         * @param var 
         * @param index 
         * @param t 
         */
        void createConstraints(const std::shared_ptr<State>& occupancy_state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t);

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

        Pair<std::shared_ptr<Action>,double> createRelaxedSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);
        Pair<std::shared_ptr<Action>,double> createFullSawtooth(const std::shared_ptr<ValueFunction>& vf,const std::shared_ptr<State> &state, number t);

        virtual void createInitialConstrainte(const std::shared_ptr<State> &state, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t) =0;
        virtual void createSawtoothBigM(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_one_step_uncompressed_occupancy_state, double probability, double difference, IloEnv &env, IloRangeArray &con, IloNumVarArray &var, number &index, number t) = 0;
        virtual void createSawtoothIloIfThen(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<State> &next_hidden_state, const std::shared_ptr<Observation> &next_observation, const std::shared_ptr<JointHistoryInterface> &next_joint_history, const std::shared_ptr<State> &next_state, double probability, double difference, IloEnv &env, IloModel &model, IloNumVarArray &var,number t) = 0;
        
        double getQValueRealistic(const std::shared_ptr<State> &compressed_occupancy_state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, std::shared_ptr<State> next_hidden_state, const std::shared_ptr<Observation> next_observation, double denominator, double difference);
        double getSawtoothMinimumRatio(const std::shared_ptr<State> &state, const std::shared_ptr<JointHistoryInterface>& joint_history, const std::shared_ptr<Action>& action, const std::shared_ptr<State>& next_hidden_state, const std::shared_ptr<Observation>& next_observation, double denominator);

    };
}
