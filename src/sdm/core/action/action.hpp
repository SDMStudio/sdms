/**
 * @file action.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief The file for interface action class
 * @version 0.1
 * @date 11/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/item.hpp>

namespace sdm
{
    class DecisionRule;
    class JointDeterministicDecisionRule;

    /**
     * @brief A public interface for actions. 
     * Any class inheriting from this interface will be considered as generic action for algorithms.
     * Consider sections [Theoritical Background](https://aldavid.gitlabpages.inria.fr/sdms/tutorials/theory.html) and [Algorithms](https://aldavid.gitlabpages.inria.fr/sdms/tutorials/algorithms/) for more information.   
     */
    class Action : public Item
    {
    public:
        virtual ~Action() {}
        
        /** @brief Cast the action into a decision rule. */
        virtual std::shared_ptr<DecisionRule> toDecisionRule();
        
        /** @brief Cast the action into a joint decision rule. */
        virtual std::shared_ptr<JointDeterministicDecisionRule> toJointDeterministicDecisionRule();
        
        virtual std::string str() const = 0;
        virtual TypeAction getTypeAction() const;
    };
} // namespace sdm