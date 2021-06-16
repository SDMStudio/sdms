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
    // class DiscreteAction;
    // class StochasticDecisionRule;
    // class JointAction;
    // class JointDeterministicDecisionRule;
    // class DeterministicDecisionRule;


    class Action : public Item
    {
    public:
        virtual ~Action() {}
        virtual std::string str() const = 0;

        // /**
        //  * @brief Transform the State in a DiscreteAction
        //  * 
        //  * @return std::shared_ptr<OccupancyStateInterface> 
        //  */
        // virtual std::shared_ptr<DiscreteAction> toDiscreteAction();

        // /**
        //  * @brief Transform the State in a JointAction
        //  * 
        //  * @return std::shared_ptr<JointAction> 
        //  */
        // virtual std::shared_ptr<JointAction> toJointAction();

        //         /**
        //  * @brief Transform the State in a StochasticDecisionRule
        //  * 
        //  * @return std::shared_ptr<StochasticDecisionRule> 
        //  */
        // virtual std::shared_ptr<StochasticDecisionRule> toStochasticDecisionRule();

        //         /**
        //  * @brief Transform the State in a JointDeterministicDecisionRule
        //  * 
        //  * @return std::shared_ptr<JointDeterministicDecisionRule> 
        //  */
        // virtual std::shared_ptr<JointDeterministicDecisionRule> toJointDeterministicDecisionRule();

        // /**
        //  * @brief Transform the State in a DeterministicDecisionRule
        //  * 
        //  * @return std::shared_ptr<DeterministicDecisionRule> 
        //  */
        // virtual std::shared_ptr<DeterministicDecisionRule> toDeterministicDecisionRule();

    };
} // namespace sdm