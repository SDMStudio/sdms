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
#include <sdm/core/state/belief_state_graph.hpp>

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
     * @brief An base occupancy MDP is a subclass of continuous state MDP where states are occupancy states. 
     * In the general case, an occupancy state refers to the whole knowledge that a central planner can have access to take decisions. But in this implementation we call occupancy state are distribution over state and joint histories .
     * 
     * @tparam TState the occupancy state type 
     * @tparam TAction the occupancy action type 
     */
    template <typename TState, typename TAction>
    class BaseOccupancyMDP : public SolvableByHSVI<TState, TAction>,
                             public GymInterface<TState, TAction>
    {
    protected:
        /**
         * @brief Type of occupancy state.
         */
        bool compress = true, keep_one_step_uncompressed = false, keep_fully_uncompressed = false;

        /**
         * @brief The problem that we want solve. 
         */
        std::shared_ptr<DiscreteDecPOMDP> dpomdp_;

        /**
         * @brief Keep initial and current states.
         */
        std::shared_ptr<TState> initial_state_, current_state_;

        /**
         * @brief Keep initial and current histories.
         */
        typename TState::jhistory_type initial_history_ = nullptr, current_history_ = nullptr;

    public:
        using state_type = TState;
        using action_type = TAction;
        // using observation_type = oObservation;

        BaseOccupancyMDP();

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP (as a filename)
         * @param hist_length the maximum length of the history
         */
        BaseOccupancyMDP(std::string, number = -1);

        /**
         * @brief Construct a new Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP 
         * @param hist_length the maximum length of the history
         */
        BaseOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP>, number = -1);

        virtual void initialize(number history_length) = 0;

        // ----------------------------------------
        // ---------- RL GymInterface -------------
        // ----------------------------------------

        /**
         * @brief Reset the environment and return initial observation.
         * 
         * @return the initial observation
         */
        TState reset();

        /**
         * @brief Do a step on the environment.
         * 
         * @param action the action to execute
         * @return the information produced. Include : next observation, rewards, episode done  
         */
        virtual std::tuple<TState, std::vector<double>, bool> step(TAction action) = 0;

        // ---------------------------------------------
        // ---------- HSVI exact interface -------------
        // ---------------------------------------------

        bool isSerialized() const;

        TState getInitialState();

        virtual TState nextState(const TState &, const TAction &, number, std::shared_ptr<HSVI<TState, TAction>>, bool) const = 0;

        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;

        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &);

        virtual double getReward(const TState &, const TAction &) const = 0;

        double getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>>, const TState &, const TAction &, number = 0) const;

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

        /**
         * @brief Get the underlying problem (i.e. the DecPOMDP formalism)
         * 
         * @return a formalism
         */
        DiscreteDecPOMDP *getUnderlyingProblem();

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
        std::shared_ptr<BeliefMDP<BeliefState<>, number, number>> toBeliefMDP();
    };
} // namespace sdm
#include <sdm/world/base/base_occupancy_mdp.tpp>