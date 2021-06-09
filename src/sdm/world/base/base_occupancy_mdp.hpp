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
     * @tparam std::shared_ptr<State> the occupancy state type 
     * @tparam std::shared_ptr<Action> the occupancy action type 
     */
    class BaseOccupancyMDP : public BeliefMDP
    {
    protected:
        /**
         * @brief Type of occupancy state.
         */
        bool compress = true, keep_one_step_uncompressed = false, keep_fully_uncompressed = false;

        /**
         * @brief Keep initial and current histories.
         */
        std::unique_ptr<JointHistory> initial_history_ = nullptr, current_history_ = nullptr;

    public:
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
        BaseOccupancyMDP(std::shared_ptr<BasePOMDP>, number = -1);

        virtual void initialize(number history_length) = 0;

        // ----------------------------------------
        // ---------- RL GymInterface -------------
        // ----------------------------------------

        /**
         * @brief Reset the environment and return initial observation.
         * 
         * @return the initial observation
         */
        std::shared_ptr<State> reset();

        /**
         * @brief Do a step on the environment.
         * 
         * @param action the action to execute
         * @return the information produced. Include : next observation, rewards, episode done  
         */
        virtual std::tuple<std::shared_ptr<State>, std::vector<double>, bool> step(std::shared_ptr<Action> action) = 0;

        // ---------------------------------------------
        // ---------- HSVI exact interface -------------
        // ---------------------------------------------

        bool isSerialized() const;

        std::shared_ptr<State> getInitialState();

        virtual std::shared_ptr<State> nextState(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number, std::shared_ptr<HSVI>, bool) const = 0;

        std::shared_ptr<State> nextState(const std::shared_ptr<State> &, const std::shared_ptr<Action> &, number = 0, std::shared_ptr<HSVI> = nullptr) const;

        std::shared_ptr<DiscreteSpace> getActionSpaceAt(const std::shared_ptr<State> &);

        virtual double getReward(const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule) const = 0;

        double getExpectedNextValue(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &occupancy_state, const std::shared_ptr<Action> &joint_decision_rule, number t = 0) const;

        /**
         * @brief Get the specific discount factor for the problem at hand
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        double getDiscount(number t = 0);

        /**
         * @brief Get the specific weighted discount factor for the problem at hand
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        double getWeightedDiscount(number t);

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
         * @param const std::shared_ptr<ValueFunction>& : the lower bound
         * @param const std::shared_ptr<ValueFunction>& : the upper bound
         * @param const std::shared_ptr<State> & s : current state
         * @param number h : horizon
         * @return std::shared_ptr<Action> 
         */
        std::shared_ptr<Action> selectNextAction(const std::shared_ptr<ValueFunction> &lb, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &s, number t);

        /**
         * @brief Get the underlying problem (i.e. the DecPOMDP formalism)
         * 
         * @return a formalism
         */
        std::shared_ptr<DecisionProcess> getUnderlyingProblem();

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