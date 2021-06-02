#pragma once

#include <sdm/types.hpp>
#include <sdm/world/base/mdp_interface.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>

namespace sdm
{
    /**
     * @brief The class for Discrete Markov Decision Processes.
     *
     */
    class MDP : public SolvableByHSVI
    {
    public:
        MDP();

        MDP(const std::shared_ptr<MDPInterface> &mdp);

        /**
         * @brief Get the initial state
         */
        std::shared_ptr<State> getInitialState();

        /**
         * @brief Get the specific discount factor for the problem at hand
         * 
         * @param number decision epoch or any other parameter 
         * @return double discount factor
         */
        double getDiscount(number t = 0);

        /**
         * @brief Get the specific weighted discount factor for the problem at hand
         * 
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
        double do_excess(double incumbent, double lb_value, double ub_value, double cost_so_far, double error, number horizon);

        /**
         * @brief Select the next action
         * 
         * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the lower bound
         * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the upper bound
         * @param const TState & s : current state
         * @param number h : horizon
         * @return TAction 
         */
        std::shared_ptr<Action> selectNextAction(const std::shared_ptr<ValueFunction> &lb, const std::shared_ptr<ValueFunction> &ub, const std::shared_ptr<State> &s, number h);

        /**
         * @brief Get the next occupancy state.
         * 
         * @param state the occupancy state
         * @param action the action state
         * @param t the timestep
         * @param hsvi a pointer on the algorithm that makes the call
         * @return the next occupancy state
         */
        std::shared_ptr<State> nextState(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0, std::shared_ptr<HSVI> hsvi = nullptr) const;

        /**
         * @brief Get the actions availables at a specific state
         * 
         * @param state the state
         * @return the action space 
         */
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const std::shared_ptr<State> &state);

        /**
         * @brief Get the reward at a given occupancy state and occupancy action 
         */
        double getReward(const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action) const;

        /**
         * @brief Get the expected next value
         * 
         * @param value_function a pointer on the value function to use to perform the calculus.
         * @param state the state on which to evaluate the next expected value *
         * @param action 
         * @param t 
         * @return double 
         */
        double getExpectedNextValue(std::shared_ptr<ValueFunction> value_function, const std::shared_ptr<State> &state, const std::shared_ptr<Action> &action, number t = 0) const;

        /**
         * @brief Get the underlying problem. For instance the underlying DecPOMDP of the OccupancyMDP or the underlying POMDP of the current BeliefMDP.  
         * 
         * @return the underlying problem 
         */
        std::shared_ptr<MDPInterface> getUnderlyingProblem();

        /**
         * @brief Check if the problem is serialized.
         * 
         * @return true if the problem is serialized.
         * @return false if the problem is not serialized.
         */
        bool isSerialized() const;

    protected:
        std::shared_ptr<MDPInterface> underlying_problem;

        std::shared_ptr<MDPInterface> getUnderlyingMDP();
    };
} // namespace sdm