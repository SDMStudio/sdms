/**
 * @file belief_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File that contains the implementation of the belief mdp process class.
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/belief_state.hpp>
#include <sdm/core/state/interface/belief_interface.hpp>
#include <sdm/core/action/action.hpp>
#include <sdm/utils/struct/recursive_map.hpp>
#include <sdm/utils/struct/graph.hpp>
#include <sdm/world/solvable_by_mdp.hpp>
#include <sdm/world/base/pomdp_interface.hpp>
#include <sdm/world/gym_interface.hpp>
#include <sdm/world/mdp.hpp>

namespace sdm
{
    /**
     * @brief The BaseBeliefMDP class is the interface contains the transformation of a the POMDP formalism in BaseBeliefMDP formalism.
     */
    template <class TBelief>
    class BaseBeliefMDP : public SolvableByMDP,
                          public GymInterface
    {
    public:
        BaseBeliefMDP();
        BaseBeliefMDP(const std::shared_ptr<POMDPInterface> &pomdp);

        std::shared_ptr<State> nextState(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr);
        std::shared_ptr<State> nextBelief(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);

        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<Observation> &observation, number t);
        std::shared_ptr<Space> getActionSpaceAt(const std::shared_ptr<State> &belief, number t = 0);

        std::shared_ptr<Observation> reset();

        std::tuple<std::shared_ptr<Observation>, std::vector<double>, bool> step(std::shared_ptr<Action> action);

        double getReward(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0);

        double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, number t = 0);

        /**
         * @brief Get the Observation Probability p(o | b', a)
         */
        double getObservationProbability(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<State> &next_belief, const std::shared_ptr<Observation> &obs, number t = 0) const;

    protected:
        std::shared_ptr<State> current_state_;

        int step_;

        bool backup = true;

        RecursiveMap<TBelief, std::shared_ptr<State>> state_space_;

        /**
         * @brief A pointer on the bag containing all nodes.
         */
        RecursiveMap<std::shared_ptr<State>, std::shared_ptr<Action>, std::shared_ptr<Observation>, double> transition_probability;

        /**
         * @brief 
         */
        std::shared_ptr<Graph<std::shared_ptr<State>, Pair<std::shared_ptr<Action>, std::shared_ptr<Observation>>>> mdp_graph_;

        std::shared_ptr<POMDPInterface> getUnderlyingPOMDP() const;

        /**
         * @brief This part of the code can be used to 
         * 
         * @param belief 
         * @param action 
         * @param obs 
         * @param t 
         * @return std::shared_ptr<BeliefInterface> 
         */
        virtual Pair<std::shared_ptr<State>, double> computeNextStateAndProba(const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation> &observation, number t = 0);
    };

    using BeliefMDP = BaseBeliefMDP<Belief>;

}
#include <sdm/world/belief_mdp.tpp>
