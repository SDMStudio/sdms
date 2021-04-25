
#pragma once
#include <set>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/value_function/initializer.hpp>
#include <sdm/utils/value_function/value_function.hpp>

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
    class MaxPlanValueFunctionLP : public MaxPlanValueFunction<TVector, TAction, TValue>
    {

    public:

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
        std::shared_ptr<TAction> greedyMaxPlane(const TVector&, const TVector&, double&, double);

        void setGreedyObjective(const TVector&, IloNumVarArray&, IloObjective&, const TVector&) const;

        void setGreedyVariables(const TVector&, std::vector<std::set<typename TVector::jhistory_type::element_type::ihistory_type>>&, IloEnv&, IloNumVarArray&) const;

    };
}
#include <sdm/utils/value_function/sawtooth_vf_copy.tpp>
