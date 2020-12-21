/**
 * @file belief_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Belief MDP formalism
 * @version 0.1
 * @date 21/12/2020
 * 
 * @copyright Copyright (c) 2020
 * 
 * Provides several transformed functions.
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/world/decpomdp.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{
    //! \class  DecPOMDP
    class BeliefMDP
    {
    protected:
        std::shared_ptr<DecPOMDP> hidden_mdp;
        // Vector current_belief;

    public:
        BeliefMDP(std::shared_ptr<DecPOMDP> underlying_pomdp);

        Vector getBelief() const;
        Vector getNextBelief(Vector belief, number action, number obs) const;

        /**
         * @fn double getReward(Vector belief, number action);
         * @brief Get transformed reward from action and belief  
         */
        double getReward(Vector belief, number action) const;

        double getObservationProbability(number action, number obs, Vector belief) const;
    };
} // namespace sdm
