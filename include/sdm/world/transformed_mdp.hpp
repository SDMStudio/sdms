/**
 * @file transformed_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief Transformed MDP abstract class
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
    //! \class  TransformedMDP

    template <typename TState, typename TAction, typename TObserv>
    class TransformedMDP
    {
    protected:
        std::shared_ptr<POSG> underlying_problem;

    public:
        TransformedMDP(std::shared_ptr<POSG> underlying_problem);

        std::shared_ptr<POSG> getUnderlyingProblem() const;
        virtual TState getNextState(TState o_state, TAction o_action, TObserv obs) const = 0;
        virtual double getReward(TState o_state, TAction o_action) const = 0;
        virtual double getObservationProbability(TAction o_action, TObserv obs, TState o_state) const = 0;


    };
} // namespace sdm
