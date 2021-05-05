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
namespace sdm{
    
    template <typename TState, typename TAction, typename TValue = double>
    class SawtoothValueFunctionLP : public DecentralizedConstraintsLP<TState,TAction,TValue>, public SawtoothValueFunction<TState,TAction,TValue>
    {

    protected:
        // //<! this stores default hyperplan
        // std::shared_ptr<mdp_value_function> mdp_vf;

        double getUpperBound(const TState& , number ) ;

        // double getSawtoothValueAt(const TState& , const TState&) const;

        double getSawtoothUpperBound(const TState& , typename TState::jhistory_type , TAction , number );

        // double getSawtoothValueAt(const TState&, double , const TState& , double ) const;

        // double getSawtoothMinimumRatio(const std::shared_ptr<occupancy_map<jhistory>>&, jhistory*, action, state, jhistory*, const std::shared_ptr<occupancy_map<jhistory>>&) const;

        void setGreedySawtoothConstraints(const TState& , const TState& , IloEnv& , IloRangeArray& , IloNumVarArray& , number& , number , state , typename TState::jhistory_type, double , bool ) ;

    public:
        SawtoothValueFunctionLP();
        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<TState, TAction>> , number , std::shared_ptr<Initializer<TState, TAction>> );
        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<TState, TAction>> , number  = 0, TValue  = 0.);

        TAction greedySawtooth(const TState&, double , double&, number);

        void setGreedyObjective(IloObjective& , IloNumVarArray& , number) ;

        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
        void setGreedySawtooth(const TState&, IloEnv&, IloRangeArray&, IloNumVarArray& , number&, number  ) ;

        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        void setGreedySawtooth(const TState&, IloEnv&, IloRangeArray&, IloNumVarArray& , number&, number  ) ;


        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
        void setGreedyUpperbound(const TState&, IloEnv& , IloRangeArray&, IloNumVarArray& , number& , number) ;

        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        void setGreedyUpperbound(const TState&, IloEnv& , IloRangeArray&, IloNumVarArray& , number& , number) ;

        void setGreedyVariables(const TState&, std::unordered_map<agent, std::unordered_set<typename TState::jhistory_type::element_type::ihistory_type>>&, IloEnv& , IloNumVarArray&, double , double, number  ) ;

        void updateValueAt(const TState &, number );
    };


    template <class TAction, class TValue>
    class SawtoothValueFunctionLP<number, TAction, TValue> : public SawtoothValueFunction<number, TAction, TValue>
    {
    public:

        SawtoothValueFunctionLP()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }
        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<number, TAction>>, number = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = number.");
        }

        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<number, TAction>> problem, number horizon, std::shared_ptr<Initializer<number, TAction>> initializer) : SawtoothValueFunction<number, TAction, TValue>(problem, horizon, initializer)
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

    template <class TAction, class TValue >
    class SawtoothValueFunctionLP<SerializedState, TAction, TValue> : public SawtoothValueFunction<SerializedState, TAction, TValue>
    {
    public:

        SawtoothValueFunctionLP()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }
        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>>, number = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = SerializedState.");
        }

        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>> problem, number horizon, std::shared_ptr<Initializer<SerializedState, TAction>> initializer) :SawtoothValueFunction<SerializedState, TAction, TValue>(problem, horizon, initializer)
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

    template <class TAction, class TValue >
    class SawtoothValueFunctionLP<BeliefState, TAction, TValue> : public SawtoothValueFunction<BeliefState, TAction, TValue>
    {
    public:

        SawtoothValueFunctionLP()
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }
        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<BeliefState, TAction>>, number = 0, TValue = 0.)
        {
            throw sdm::exception::Exception("Sawtooth_LP cannot be used for State = BeliefState.");
        }

        SawtoothValueFunctionLP(std::shared_ptr<SolvableByHSVI<BeliefState, TAction>> problem, number horizon, std::shared_ptr<Initializer<BeliefState, TAction>> initializer) :SawtoothValueFunction<BeliefState, TAction, TValue>(problem, horizon, initializer)
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
#include <sdm/utils/value_function/sawtooth_vf_with_lp.tpp>
