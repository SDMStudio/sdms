

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
        DecentralizedConstraintsLP(std::shared_ptr<SolvableByHSVI<TVector, TAction>>);

        /**
         * @brief Get the decentralized (joint) decision rule from the result
         * @param const IloCplex&
         * @param const IloNumVarArray&
         * @param const TVector&
         * @return TAction 
         */
        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
        TAction getDecentralizedVariables(const IloCplex&, const IloNumVarArray&, const TVector&);

        /**
         * @brief Set decentralized variables 
         * @param const TVector&
         * @param const std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number&
         */
        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
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
        template <typename T, std::enable_if_t<std::is_same_v<OccupancyState<>, T>, int> = 0>
        void setDecentralizedConstraints(const TVector&, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&, IloEnv&, IloRangeArray&, IloNumVarArray&, number&);

        /**
         * @brief Get the decentralized (joint) decision rule from the result
         * @param const IloCplex&
         * @param const IloNumVarArray&
         * @param const TVector&
         * @return TAction 
         */
        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        TAction getDecentralizedVariables(const IloCplex&, const IloNumVarArray&, const TVector&);

        /**
         * @brief Set decentralized variables 
         * @param const TVector&
         * @param const std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&
         * @param const IloEnv&
         * @param const IloNumVarArray&
         * @param const number&
         */
        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
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
        template <typename T, std::enable_if_t<std::is_same_v<SerializedOccupancyState<>, T>, int> = 0>
        void setDecentralizedConstraints(const TVector&, std::unordered_map<agent, std::unordered_set<typename TVector::jhistory_type::element_type::ihistory_type>>&, IloEnv&, IloRangeArray&, IloNumVarArray&, number&);

    protected : 
        std::shared_ptr<SolvableByHSVI<TVector, TAction>> world_;

    };
}
#include <sdm/utils/value_function/decentralized_constraints_with_lp.tpp>
