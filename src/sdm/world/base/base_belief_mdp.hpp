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

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_mmdp.hpp>

namespace sdm
{
    /**
     * @brief The BaseBeliefMDP class is the interface that enables solving Discret POMDP using HSVI algorithm.
     * 
     * @tparam TBelief the belief type
     * @tparam TAction the action type
     */
    template <typename TBelief = BeliefState<>, typename TAction = number, typename TObservation = number>
    class BaseBeliefMDP : public SolvableByHSVI<TBelief, TAction>,
                          public GymInterface<TBelief, TAction>,
                          public std::enable_shared_from_this<BaseBeliefMDP<TBelief, TAction, TObservation>>
    {
    public:
        using state_type = TBelief;
        using action_type = TAction;
        using observation_type = TBelief;

        BaseBeliefMDP();
        BaseBeliefMDP(std::shared_ptr<DiscretePOMDP> underlying_pomdp);
        BaseBeliefMDP(std::string underlying_pomdp);

        TBelief reset();
        std::tuple<TBelief, std::vector<double>, bool> step(TAction action);

        TBelief getInitialState();
        virtual TBelief nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const = 0;
        virtual TBelief nextState(const TBelief &belief, const TAction &action, number t, std::shared_ptr<HSVI<TBelief, TAction>> hsvi) const = 0;

        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TBelief &ostate = TBelief());

        virtual double getReward(const TBelief &belief, const TAction &action) const = 0;
        double getExpectedNextValue(std::shared_ptr<ValueFunction<TBelief, TAction>> value_function, const TBelief &belief, const TAction &action, number t) const;

        /**
         * @brief Get the Observation Probability p(o | b, a)
         */
        virtual double getObservationProbability(const TBelief &, const TAction &action, const TObservation &obs, const TBelief &belief) const = 0;

        bool isSerialized() const;
        DiscretePOMDP *getUnderlyingProblem();

        /**
         * @brief Get the corresponding Markov Decision Process. It corresponds to the reformulation of the Belief MDP in a MDP where the blief state space is the state space. 
         * 
         * @return a belief MDP
         */
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. In this particular case, it will return the current MDP
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BaseBeliefMDP<TBelief, TAction, TObservation>> toBeliefMDP();

        /**
         * @brief Get the specific discount factor for the problem at hand
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        double getDiscount(number = 0);

        /**
         * @brief Get the specific weighted discount factor for the problem at hand
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        double getWeightedDiscount(number);

        /**
         * @brief Compute the excess of the HSVI paper. It refers to the termination condition.
         * 
         * @param double : incumbent 
         * @param double : lb value
         * @param double : ub value
         * @param double : cost_so_far 
         * @param double : error 
         * @param number : horizon 
         * @return double 
         */
        double do_excess(double, double, double, double, double, number);

        /**
         * @brief Select the next action
         * 
         * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the lower bound
         * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the upper bound
         * @param const TState & s : current state
         * @param number h : horizon
         * @return TAction 
         */
        TAction selectNextAction(const std::shared_ptr<ValueFunction<TBelief, TAction>> &, const std::shared_ptr<ValueFunction<TBelief, TAction>> &, const TBelief &, number);

    protected:
        std::shared_ptr<DiscretePOMDP> pomdp_;
        TBelief initial_state_;
        TBelief current_state_;
    };
}
#include <sdm/world/base/base_belief_mdp.tpp>
