#pragma once

#include <vector>
#include <unordered_map>
#include <ilcplex/ilocplex.h>

#include <sdm/utils/value_function/decentralized_constraints_with_lp.hpp>
#include <sdm/utils/value_function/sawtooth_vf.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/tabular_value_function.hpp>

//! This class provides getter and setter methods to characterize convexhull of points using MILP.
//!

//! \namespace  sdm
//!
//! Namespace grouping all tools required for sequential decision making.
namespace sdm
{

    template <typename TState, typename TAction, typename TValue = double>
    class SawtoothValueFunctionLPRelaxed : public DecentralizedConstraintsLP<TState, TAction, TValue>, public SawtoothValueFunction<TState, TAction, TValue>
    {

    public:

        SawtoothValueFunctionLPRelaxed();

        /**
         * @brief Construct a new Sawtooth Value Function L P object
         * 
         * @param std::shared_ptr<SolvableByHSVI<TState, TAction>> : problem 
         * @param number : horizon 
         * @param std::shared_ptr<Initializer<TState, TAction>> : initializer
         */
        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<TState, TAction>> , number , std::shared_ptr<Initializer<TState, TAction>>);
        
        /**
         * @brief Construct a new Sawtooth Value Function L P object
         * 
         * @param std::shared_ptr<SolvableByHSVI<TState, TAction>> : problem 
         * @param number : horizon 
         * @param TValue : default value for initializer
         */
        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<TState, TAction>> , number  = 0, TValue  = 0.);

        /**
         * @brief Get the best action to do at a state
         * 
         * @param state the state
         * @return the best action
         */
        TAction getBestAction(const TState &, number = 0);

        /**
         * @brief Returns the greedy decision rule for the current occupancy state
         * 
         * @param const TState & : compressed_occupancy_state 
         * @param number : t 
         * @return Pair<TAction,double> 
         */
        Pair<TAction,double> getGreedySawtooth(const TState &, number );


        /**
        * @brief Returns the greedy decision rule for the current occupancy state and an occupancy state from the point set in t+1 
        * 
        * @param const TState& : current occupancy state
        * @param Pair<TState,double> : an occupancy state from the point set in t+1 
        * @param number : time step
        * @param double : limit inf for the objective function
        * @param double : limit sup for the objective function
        * 
        * @return TAction 
        */
        Pair<TAction,double> greedySawtooth(const TState&,const Pair<TState,double>& , number, double = -IloInfinity, double = 0);

        /**
         * @brief Create the variables
         * 
         * @param const TState& : current state
         * @param std::unordered_map<agent, std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type>>& : 
         * @param IloEnv& : env 
         * @param IloNumVarArray& : var 
         * @param double : limit inf for the objective function
         * @param double : limit sup for the objective function
         * @param number : time step
         * 
         * @warning The param who resctric the lower bound is not implemented for the moment
         */
        void setGreedyVariables(const TState&, std::unordered_map<agent, std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type>>&, IloEnv& , IloNumVarArray&, double , double, number  ) ;

        /**
         * @brief Set the objective function
         * 
         * @param const TState& : current state : 
         * @param IloObjective& : obj
         * @param IloNumVarArray& : var
         * @param number : time step
         */
        template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int> = 0>
        void setGreedyObjective(const TState&, IloObjective& , IloNumVarArray& , number) ;

        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        void setGreedyObjective(const TState&, IloObjective& , IloNumVarArray& , number) ;

       /**
        * @brief Built sawtooth constraint 
        * 
        * @param const TState & : current occupancy state
        * @param const TState & : an occupancy state from the point set in t+1 
        * @param IloModel & :
        * @param IloEnv & : 
        * @param IloRangeArray & :
        * @param IloNumVarArray & :
        * @param number & : index 
        * @param number : time step
        */
        template <typename T, std::enable_if_t<std::is_any<T, OccupancyState<>, OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>>::value, int> = 0>
        void setGreedySawtooth(const TState &,const Pair<TState,double> &, IloModel &, IloEnv &, IloRangeArray &, IloNumVarArray &, number &, number);

        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        void setGreedySawtooth(const TState&,const Pair<TState,double> &, IloModel&, IloEnv&, IloRangeArray&, IloNumVarArray& , number&, number  ) ;

        void updateValueAt(const TState &, number );

        TValue getValueAt(const TState &, number );

    protected:

        /**
         * @brief Get the Sawtooth Minimum Ratio  i.e. \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)}
         * 
         * @param const TState& : one_step_uncompressed_occupancy_state
         * @param typename TState::jhistory_type : joint history
         * @param typename TAction::output_type : action 
         * @param typename TState::state_type : next_hidden_state
         * @param typename TState::observation_type : next_observation
         * @param const TState& : next_one_step_uncompressed_occupancy_state+1
         * 
         * @return double : Ratio
         */
        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
        double getSawtoothMinimumRatio(const TState &, typename TState::jhistory_type, typename TAction::output_type, typename TState::state_type, typename TState::observation_type, double);

        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int> = 0>
        double getSawtoothMinimumRatio(const TState &, typename TState::jhistory_type, typename TAction::output_type, typename TState::state_type, typename TState::observation_type, double);

        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        double getSawtoothMinimumRatio(const TState&, typename TState::jhistory_type , typename TAction::output_type , typename TState::state_type , typename TState::observation_type , double ) ;

        /**
         * @brief Return the \sum_x s(x,o) Q_MDP(x,u)
         * @warning : should depend on the initializer used, ie the relaxation. 
         * @param const TState& 
         * @param typename TState::jhistory_type  : joint history
         * @param typename typename TAction::output_type  : action 
         * @param number : time step
         * @return double 
         */
        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
        double getQValueRelaxation(const TState &, typename TState::jhistory_type, typename TAction::output_type, number);

        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<BeliefStateGraph_p<number, number>, JointHistoryTree_p<number>>, T>, int> = 0>
        double getQValueRelaxation(const TState &, typename TState::jhistory_type, typename TAction::output_type, number);

        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        double getQValueRelaxation(const TState &, typename TState::jhistory_type, typename TAction::output_type, number);

        /**
         * @brief Build sawtooth constraints  Q(k,s,o,u,y,z, diff, t ) = (v_k - V_k) \frac{\sum_{x} s(x,o) * p(x,u,z,y)}}{s_k(y,<o,z>)},  \forall a(u|o)
         * 
         * @param const TState& : current compressed occupacy state
         * @param typename TState::jhistory_type : joint_history 
         * @param typename TAction::output_type : action 
         * @param typename TState::state_typ : next_hidden_state 
         * @param typename TState::observation_type : next_observation 
         * @param const TState : next_one_step_uncompressed_occupancy_state 
         * @param double : difference i.e. (v_k - V_k)
         * @return double 
         */
        double getQValueRealistic(const TState&, typename TState::jhistory_type, typename TAction::output_type, typename TState::state_type, typename TState::observation_type, double, double );

    };

    template <class TAction, class TValue>
    class SawtoothValueFunctionLPRelaxed<number, TAction, TValue> : public SawtoothValueFunction<number, TAction, TValue>
    {
    public:
        SawtoothValueFunctionLPRelaxed()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }
        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<number, TAction>>, number = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }

        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<number, TAction>> problem, number horizon, std::shared_ptr<Initializer<number, TAction>> initializer) : SawtoothValueFunction<number, TAction, TValue>(problem, horizon, initializer)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }
        void initialize(TValue, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }

        TValue getValueAt(const number &, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }

        void updateValueAt(const number &, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }

        std::vector<number> getSupport(number)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }

        std::string str()
        {
            return "Sawtooth_LP";
        }
    };

    template <class TAction, class TValue>
    class SawtoothValueFunctionLPRelaxed<SerializedState, TAction, TValue> : public SawtoothValueFunction<SerializedState, TAction, TValue>
    {
    public:
        SawtoothValueFunctionLPRelaxed()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }
        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>>, number = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }

        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>> problem, number horizon, std::shared_ptr<Initializer<SerializedState, TAction>> initializer) : SawtoothValueFunction<SerializedState, TAction, TValue>(problem, horizon, initializer)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }
        void initialize(TValue, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }

        TValue getValueAt(const SerializedState &, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }

        void updateValueAt(const SerializedState &, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }

        std::vector<SerializedState> getSupport(number)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }

        std::string str()
        {
            return "Sawtooth_LP";
        }
    };

    template <class TAction, class TValue>
    class SawtoothValueFunctionLPRelaxed<BeliefState, TAction, TValue> : public SawtoothValueFunction<BeliefState, TAction, TValue>
    {
    public:
        SawtoothValueFunctionLPRelaxed()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }
        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<BeliefState, TAction>>, number = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }

        SawtoothValueFunctionLPRelaxed(std::shared_ptr<SolvableByHSVI<BeliefState, TAction>> problem, number horizon, std::shared_ptr<Initializer<BeliefState, TAction>> initializer) : SawtoothValueFunction<BeliefState, TAction, TValue>(problem, horizon, initializer)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }
        void initialize(TValue, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }

        TValue getValueAt(const BeliefState &, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }

        void updateValueAt(const BeliefState &, number = 0)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }

        std::vector<BeliefState> getSupport(number)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }

        std::string str()
        {
            return "Sawtooth_LP";
        }
    };
}
#include <sdm/utils/value_function/sawtooth_vf_with_lp_relaxed.tpp>