/**
 * @file occupancy_mdp.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief 
 * @version 1.0
 * @date 03/02/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/core/joint.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/state.hpp>
#include <sdm/core/state/occupancy_state.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/discrete_mdp.hpp>
#include <sdm/world/discrete_pomdp.hpp>
#include <sdm/world/discrete_decpomdp.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/joint_det_decision_rule.hpp>

/**
 * @namespace  sdm
 * @brief Namespace grouping all tools required for sequential decision making.
 */
namespace sdm
{

    /**
     * @brief An occupancy MDP is a subclass of continuous state MDP where states are occupancy states. 
     * In the general case, an occupancy state refers to the whole knowledge that a central planner can have access to take decisions. But in this implementation we call occupancy state are distribution over state and joint histories .
     * 
     * @tparam TState the occupancy state type 
     * @tparam TAction the occupancy action type 
     */
    template <typename TState = OccupancyState<number, JointHistoryTree_p<number>>,
              typename TAction = JointDeterministicDecisionRule<HistoryTree_p<number>, number>>
    class BaseOccupancyMDP : public SolvableByHSVI<TState, TAction>,
                             public GymInterface<TState, TAction>
    {
    protected:
        std::shared_ptr<DiscreteDecPOMDP> dpomdp_;
        std::shared_ptr<TState> initial_state_, current_state_;
        typename TState::jhistory_type initial_history_ = nullptr, current_history_ = nullptr;

    public:
        using state_type = TState;
        using action_type = TAction;
        // using observation_type = oObservation;

        OccupancyMDP();

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP (as a filename)
         * @param hist_length the maximum length of the history
         */
        OccupancyMDP(std::string, number = -1);

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP 
         * @param hist_length the maximum length of the history
         */
        OccupancyMDP(std::shared_ptr<DiscreteDecPOMDP>, number = -1);

        // ---------- RL GymInterface -------------
        TState reset();
        TState &getState();
        std::tuple<TState, std::vector<double>, bool> step(TAction);

        // ---------- HSVI exact interface -------------
        bool isSerialized() const;
        DiscreteDecPOMDP *getUnderlyingProblem();

        TState getInitialState();
        TState nextState(const TState &, const TAction &, number, std::shared_ptr<HSVI<TState, TAction>>, bool) const;
        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;

        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &);

        double getReward(const TState &, const TAction &) const;
        double getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>>, const TState &, const TAction &, number = 0) const;

        // ---------- Other -------------
        /**
         * @brief Get the corresponding Markov Decision Process. 
         * 
         * @return std::shared_ptr<DiscreteMDP> 
         */
        std::shared_ptr<DiscreteMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * @warning The above comment is wrong!!!
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP();

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
        TAction selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>> &lb, const std::shared_ptr<ValueFunction<TState, TAction>> &ub, const TState &s, number h);
    };
} // namespace sdm
#include <sdm/world/occupancy_mdp.tpp>