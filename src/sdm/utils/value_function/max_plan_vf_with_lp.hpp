
#pragma once
#include <set>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>

#include <sdm/utils/value_function/decentralized_constraints_with_lp.hpp>
#include <sdm/utils/value_function/max_plan_vf.hpp>
#include <ilcplex/ilocplex.h>
/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief 
     * 
     * @tparam TState type of hyperplan representation. Must implement sdm::VectorImpl interface.
     * @tparam TValue value type (default : double)
     */
    template <typename TVector, typename TAction, typename TValue = double>
    class MaxPlanValueFunctionLP : public MaxPlanValueFunction<TVector, TAction, TValue>, public DecentralizedConstraintsLP<TVector, TAction, TValue>
    {
    public:

        MaxPlanValueFunctionLP();
        MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> , int , std::shared_ptr<Initializer<TVector, TAction>> );
        MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>> , int = 0, TValue = 0.);

        /*
        *  \fn    std::shared_ptr<Action> greedy(const std::shared_ptr<State>&, number, double&);
        *  \param const std::shared_ptr<State>&               : current occupancy state
        *  \param const std::shared_ptr<State>&               : identifier of the hyperplan to use
        *  \param double&                                     : the reference of the value to be returned
        *  \brief Returns the greedy decision rule for the current occupancy state and the prescribed hyperplan
        * Maximize \sum_{x,o,u} A(u|o) s(x,o)  [ r(x,u) + \gamma \sum_{x_,z_} P(x_,z_|x,u) * \alpha(x_,o_)  ]
        * Subject to:
        *       A(u|o) <= A_i(u_i|o_i)
        *       A(u|o) >= \sum_i A_i(u_i|o_i) + 1 - n
        *       \sum_{u_i} A_i(u_i|o_i) = 1
        */
        TAction greedyMaxPlane(const TVector&, const TVector&, double&, double);

        void setGreedyObjective(const TVector&, IloNumVarArray&, IloObjective&, const TVector&); // J'ai enlever le const pour le moment, car j'ai besoin de under_problem

        void setGreedyVariables(const TVector&, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&, IloEnv&, IloNumVarArray&); // std::vector<std::set<typename TVector::jhistory_type::element_type::ihistory_type>>

        void updateValueAt(const TVector &, number );

        TVector backup_operator(const TVector &, number);
    };


    // a supprimer imperativement avant la publication JMLR

    template <class TAction, class TValue>
    class MaxPlanValueFunctionLP<number, TAction, TValue> : public MaxPlanValueFunction<number, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<number, TAction>>, number, std::shared_ptr<Initializer<number, TAction>>)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = number.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = number.");
        }
        void initialize(TValue, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = number.");
        }

        TValue getValueAt(const number &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = number.");
        }

        void updateValueAt(const number &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = number.");
        }

        std::vector<number> getSupport(number)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = number.");
        }

        std::string str()
        {
            return "MaxPlanLP";
        }
    };

    template <class TAction, class TValue>
    class MaxPlanValueFunctionLP<SerializedState, TAction, TValue> : public MaxPlanValueFunction<SerializedState, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<SerializedState, TAction>>, number, std::shared_ptr<Initializer<SerializedState, TAction>>)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = SerializedState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = SerializedState.");
        }
        void initialize(TValue, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = SerializedState.");
        }

        TValue getValueAt(const SerializedState &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = SerializedState.");
        }

        void updateValueAt(const SerializedState &, number = 0)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = SerializedState.");
        }

        std::vector<SerializedState> getSupport(number)
        {
            throw sdm::exception::Exception("MaxPlanLP cannot be used for State = SerializedState.");
        }

        std::string str()
        {
            return "MaxPlanLP";
        }
    };

    template <class TAction, class TValue>
    class MaxPlanValueFunctionLP<BeliefState, TAction, TValue> : public MaxPlanValueFunction<BeliefState, TAction, TValue>
    {
    public:
        MaxPlanValueFunctionLP(std::shared_ptr<SolvableByHSVI<BeliefState, TAction>> , int , std::shared_ptr<Initializer<BeliefState, TAction>> )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        void initialize()
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }
        void initialize(TValue , number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        TValue getValueAt(const BeliefState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        void updateValueAt(const BeliefState &, number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        std::vector<BeliefState> getSupport(number )
        {
            throw sdm::exception::Exception("MaxPlanVFSerialized cannot be used for State = BeliefState.");
        }

        std::string str()
        {
            return "MaxPlanVFSerialized";
        }
    };

}
#include <sdm/utils/value_function/max_plan_vf_with_lp.tpp>
