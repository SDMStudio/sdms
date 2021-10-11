/**
 * @file solvable_by_hsvi.hpp
 * @author David Albert (david.albert@insa-lyon.fr)
 * @brief File defining the interface SolvableByHSVI.
 * @version 1.0
 * @date 07/07/2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <sdm/types.hpp>
#include <sdm/exception.hpp>
#include <sdm/core/space/space.hpp>
#include <sdm/world/solvable_by_dp.hpp>
#include <sdm/world/base/mdp_interface.hpp>

namespace sdm
{
    class HSVI;
    class ValueFunction;

    /**
     * @brief Public interface for problems that are solvable with the HSVI algorithm (i.e. belief MDP, occupancy MDP, occupancy Game, etc).
     * 
     * HSVI can be used to solve all problem implementing this interface.
     * 
     */
    class SolvableByHSVI : public SolvableByDP
    {
    public:
        virtual ~SolvableByHSVI() {}

        /**
         * @brief Select the next state.
         * 
         * @param state the state
         * @param action the action
         * @param t the timestep
         * @param hsvi a pointer on the algorithm that makes the call
         * @return the next state
         */
        // virtual std::shared_ptr<State> nextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0, const std::shared_ptr<HSVI> &hsvi = nullptr) = 0;
        // virtual std::shared_ptr<State> nextStateDistribution(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const = 0;

        virtual Pair<std::shared_ptr<State>, double> getNextState(const std::shared_ptr<ValueFunction> &value_function, const std::shared_ptr<State> &belief, const std::shared_ptr<Action> &action, const std::shared_ptr<Observation>& observation, number t) = 0;

        /**
         * @brief Get the well defined underlying problem. 
         * Some problems are solvable by DP algorithms even if they are not well defined. Usually, they simply are reformulation of an underlying well defined problem. 
         * For instance, the underlying DecPOMDP of the OccupancyMDP or the underlying POMDP of the current BeliefMDP.  
         * 
         * @return the underlying problem 
         */
        virtual const std::shared_ptr<MDPInterface> &getUnderlyingProblem() const = 0;

        /**
         * @brief Check if the problem is serialized.
         * 
         * @return true if the problem is serialized.
         * @return false if the problem is not serialized.
         */
        virtual bool isSerial() const = 0;


        /**
         * @brief Get the specific weighted discount factor for the problem at hand
         * 
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        virtual double getWeightedDiscount(number t) = 0;

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
        virtual double do_excess(double incumbent, double lb_value, double ub_value, double cost_so_far, double error, number t) = 0;

        /**
         * @brief Select the next action
         * 
         * @param lb the lower bound
         * @param ub the upper bound
         * @param state the current state
         * @param t the timestep
         * @return the selected action
         */
        virtual Pair<std::shared_ptr<Action>, double> selectNextAction(const std::shared_ptr<ValueFunction> &lb, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &state, number t) = 0;
    };
} // namespace sdm