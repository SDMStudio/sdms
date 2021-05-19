#pragma once

#include <sdm/types.hpp>

#include <sdm/core/state/serialized_state.hpp>
#include <sdm/core/state/serialized_occupancy_state.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
    /**
     * @brief An Serialized occupancy MDP is a subclass of continuous state MDP where states are seriliazed occupancy states and the resolution is serialized. 
     * In the general case, a Serialized occupancy state refers to the knowledge that a central planner can have access to take decisions at a precise agent. 
     * But in this implementation we call serialized occupancy state a distribution over serialized state and joint histories .
     * 
     * @tparam TState refers to an serialized occupancy state type 
     * @tparam TAction refers to a occupancy action type 
     */
    template <typename TState = SerializedOccupancyState<SerializedState, JointHistoryTree_p<number>>, typename TAction = DeterministicDecisionRule<HistoryTree_p<number>, number>>
    class BaseSerializedOccupancyMDP : public SolvableByHSVI<TState, TAction>
    {
    protected:

        /**
         * @brief Problem that we want to solve
         * 
         */
        std::shared_ptr<SerializedMPOMDP> serialized_mpomdp_;

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

        BaseSerializedOccupancyMDP();

        /**
         * @brief Construct a new Serial Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP (as a filename)
         * @param hist_length the maximum length of the history
         */
        BaseSerializedOccupancyMDP(std::string, number = -1);

        /**
         * @brief Construct a new Serial Occupancy MDP  
         * 
         * @param underlying_dpomdp the underlying DecPOMDP (as a filename)
         * @param hist_length the maximum length of the history
         */
        BaseSerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP>, number = -1);

        virtual void initialize(number history_length) = 0;

        /**
         * @brief Check if the problem is serialized.
         * 
         * @return true
         */
        bool isSerialized() const;

        /**
         * @brief Get the actions availables at a specific state
         * 
         * @param state the state
         * @return the action space 
         */
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &);

        SerializedMPOMDP *getUnderlyingProblem();

        /**
         * @brief Get the initial state
         */
        TState getInitialState();
        
        virtual TState nextState(const TState &, const TAction &, number, std::shared_ptr<HSVI<TState, TAction>>, bool) const = 0;
        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;

        virtual double getReward(const TState &, const TAction &) const = 0;
        double getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>>, const TState &, const TAction &, number = 0) const;

        std::shared_ptr<SerializedMMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. It corresponds to the reformulation of the original POMP in a MDP where the state space is the space of beliefs. 
         * 
         * @return a belief MDP
         */
        std::shared_ptr<SerializedBeliefMDP<SerializedBeliefState,number,Joint<number>>> toBeliefMDP();

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
        TAction selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>>& lb, const std::shared_ptr<ValueFunction<TState, TAction>>& ub, const TState &s, number h);

    };
} // namespace sdm
#include <sdm/world/base/base_serial_occupancy_mdp.tpp>
