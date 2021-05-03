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
    class SerializedOccupancyMDP : public SolvableByHSVI<TState, TAction>
    {
    protected:
        std::shared_ptr<SerializedMPOMDP> serialized_mpomdp_;
        std::shared_ptr<TState> initial_state_, current_state_;
        typename TState::jhistory_type initial_history_ = nullptr, current_history_ = nullptr;

        /**
         * @brief Compute the next serial occupancy state for a non-terminal agent, i.e., expanding by the last action every entry.
         * @param const TState & serial occupancy state
         * @param const TAction & individual decision rule 
         * @return TState 
         */
        TState nextStateSerialStep(const TState &, const TAction &) const;

    public:
        using state_type = TState;
        using action_type = TAction;

        SerializedOccupancyMDP();
        SerializedOccupancyMDP(std::string, number = -1);
        SerializedOccupancyMDP(std::shared_ptr<DiscreteDecPOMDP>, number = -1);

        bool isSerialized() const;
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TState &);

        SerializedMPOMDP *getUnderlyingProblem();

        TState getInitialState();
        TState nextState(const TState &, const TAction &, number, std::shared_ptr<HSVI<TState, TAction>>, bool) const;
        TState nextState(const TState &, const TAction &, number = 0, std::shared_ptr<HSVI<TState, TAction>> = nullptr) const;

        double getReward(const TState &, const TAction &) const;
        double getExpectedNextValue(std::shared_ptr<ValueFunction<TState, TAction>>, const TState &, const TAction &, number = 0) const;

        std::shared_ptr<SerializedMMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. It corresponds to the reformulation of the original POMP in a MDP where the state space is the space of beliefs. 
         * 
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
        TAction selectNextAction(const std::shared_ptr<ValueFunction<TState, TAction>>& lb, const std::shared_ptr<ValueFunction<TState, TAction>>& ub, const TState &s, number h);

    };
} // namespace sdm
#include <sdm/world/serialized_occupancy_mdp.tpp>
