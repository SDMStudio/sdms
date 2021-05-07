
#pragma once
#include <memory>
#include <unordered_map>

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
    class VarNaming 
    {
    public:

        /**
         * @brief Set pair name identifier for a given variable 
         * @param const std::string& name
         * @param number identifier
         */
        void setNumber(const std::string& ,number);

        /**
         * @brief Get the name of a free variable  
         * @param number identifier of a variable 
         * @return std::string  name 
         */
        std::string getVarNameWeight(number);

        /**
         * @brief Get the name of a free variable  
         * @param number identifier of a variable 
         * @return std::string  name 
         */
        std::string getVarNameWeight(const TVector &);

        /**
         * @brief Get the name associated with a pair of action and joint history 
         * @param action 
         * @param typename TVector::jhistory_type
         * @return std::string name 
         */
        std::string getVarNameJointHistoryDecisionRule(int, typename TVector::jhistory_type );
        
        /**
         * @brief Get the name associated with a pair of action and joint history 
         * @param action 
         * @param typename TVector::jhistory_type
         * @return std::string name 
         */
        std::string getVarNameJointHistoryDecisionRule(typename TAction::output_type, typename TVector::jhistory_type );
        

        /**
         * @brief Get the name associated with a pair of action and individual history 
         * @param action 
         * @param typename TVector::jhistory_type::element_type::ihistory_type
         * @param agent
         * @return std::string name 
         */
        std::string getVarNameIndividualHistoryDecisionRule(int, typename TVector::jhistory_type::element_type::ihistory_type , agent );
        
        /**
         * @brief Get the name associated with a pair of action and individual history 
         * @param action 
         * @param typename TVector::jhistory_type::element_type::ihistory_type
         * @param agent
         * @return std::string name 
         */
        std::string getVarNameIndividualHistoryDecisionRule(typename TAction::output_type, typename TVector::jhistory_type::element_type::ihistory_type , agent );

        /**
         * @brief Get the identifier associated with a given name 
         * @param const std::string& name
         * @return number  identifier
         */
        number getNumber( const std::string& );

        /**
         * @brief Get the Var Name Weighted State Joint History object
         * @warning const TVector& should be something like const std::shared_ptr<TVector>& or  const TVector*&
         * @return std::string 
         */
        std::string getVarNameWeightedStateJointHistory(const TVector& , typename TVector::state_type , typename TVector::jhistory_type );

        std::string getVarNameWeightedStateJointHistory(const std::shared_ptr<TVector>& , typename TVector::state_type , typename TVector::jhistory_type );
    
    protected:

        /**
         * @brief mapping from variable names to variable identifiers 
         */
        std::unordered_map<std::string, number> variables;

    };
}

#include <sdm/utils/value_function/variable_naming.tpp>
