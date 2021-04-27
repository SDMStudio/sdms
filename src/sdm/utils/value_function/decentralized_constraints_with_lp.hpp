

#pragma once
#include <ilcplex/ilocplex.h>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/utils/value_function/variable_naming.hpp>

/**
 * @brief Namespace grouping all tools required for sequential decision making.
 * @namespace  sdm
 */
namespace sdm
{
    /**
     * @brief 
     * @tparam TVector 
     * @tparam TAction 
     * @tparam TValue 
     */
    template <typename TVector, typename TAction, typename TValue = double>
    class DecentralizedConstraintsLP  : public VarNaming<TVector, TAction, TValue>
    {
    public:

        /**
         * @brief Get the decentralized (joint) decision rule from the result
         * @param const IloCplex&
         * @param const IloNumVarArray&
         * @param const TVector&
         * @return TAction 
         */
        TAction getDecentralizedVariables(const IloCplex&, const IloNumVarArray&, const TVector&);

        /**
         * @brief Set decentralized variables 
         * @param const TVector&
         * @param const std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number&
         */
        void setDecentralizedVariables(const TVector&, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&, IloEnv&, IloNumVarArray&, number&);

        /**
         * @brief Set decentralized constraints 
         * @param const TVector& 
         * @param std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&
         * @param IloEnv& 
         * @param IloRangeArray&
         * @param IloNumVarArray&
         * @param number&
         */
        void setDecentralizedConstraints(const TVector&, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&, IloEnv&, IloRangeArray&, IloNumVarArray&, number&);
    };
}
#include <sdm/utils/value_function/decentralized_constraints_with_lp.tpp>
