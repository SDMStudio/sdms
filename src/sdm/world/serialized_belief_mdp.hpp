#pragma once

#include <sdm/types.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/world/serialized_mpomdp.hpp>

#include <sdm/core/state/beliefs.hpp>
#include <sdm/core/state/serialized_state.hpp>
#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_belief_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>

namespace sdm
{
    /**
     * @brief An Serialized MDP is a subclass of POMDP where belief are serialized beliefs. 
     * In the general case, a serialized belief refers to the whole knowledge that a central planner can have access to take decisions at the time step of a precise agent. 
     * 
     * @tparam TBelief refer to the serialized state type
     * @tparam TAction refer to the number type
     */
    template <typename TBelief = SerializedBeliefState,
              typename TAction = number,
              typename TObservation = Joint<number>>
    class SerializedBeliefMDP : public std::enable_shared_from_this<SerializedBeliefMDP<TBelief, TAction, TObservation>>,
                                public SolvableByHSVI<TBelief, TAction>
    {
    public:
        using state_type = TBelief;
        using action_type = TAction;
        using observation_type = TBelief;

        SerializedBeliefMDP();
        SerializedBeliefMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_dpomdp);
        SerializedBeliefMDP(std::shared_ptr<SerializedMPOMDP> underlying_serial_mpomdp);
        SerializedBeliefMDP(std::string underlying_dpomdp);

        /**
         * @brief Check if the problem is serialized.
         * 
         * @return true
         */
        bool isSerialized() const;

        SerializedMPOMDP *getUnderlyingProblem();

        /**
         * @brief Get the initial state
         */
        TBelief getInitialState();

        /**
         * @brief Compute the next serial belief state for a terminal agent.
         * @param const TBelief & serial belief state
         * @param const TAction & action
         * @param  const TObservation & : observation
         * @return TBelief 
         */
        TBelief nextState(const TBelief &, const TAction &, const TObservation &) const;

        /**
         * @brief Compute the next serial belief state for a non-terminal agent.
         * @param const TBelief & serial belief state
         * @param const TAction & action
         * @return TBelief 
         */
        TBelief nextStateSerialStep(const TBelief &belief, const TAction &action) const;

        /**
         * @brief Get the next occupancy state.
         * 
         * @param const TBelief & the belief state
         * @param const TAction & the action state
         * @param t the timestep
         * @param hsvi a pointer on the algorithm that makes the call
         * @return the next belief state
         */
        TBelief nextState(const TBelief &belief, const TAction &action, number t, std::shared_ptr<HSVI<TBelief, TAction>> hsvi) const;

        //Tempo à vérifier leur utilité
        // TBelief nextState(const TBelief &belief, const TAction &action, const TObservation &obs) const;

        double getObservationProbability(const TBelief &belief, const TAction &action, const TObservation &obs) const;

        /**
         * @brief Get the actions availables at a specific state
         * 
         * @param state the state
         * @return the action space 
         */
        std::shared_ptr<DiscreteSpace<TAction>> getActionSpaceAt(const TBelief &belief);

        /**
         * @brief Get the reward at a given occupancy state and occupancy action 
         * 
         * @param const TBelief & : belief state 
         * @param const TAction & : action state
         */
        double getReward(const TBelief &, const TAction &) const;

        /**
         * @brief Get the expected next value
         * 
         * @param std::shared_ptr<ValueFunction<TBelief, TAction>> a pointer on the value function to use to perform the calculus.
         * @param  const TBelief & the state on which to evaluate the next expected value *
         * @param const TAction & : action state
         * @param t : time step
         * @return double 
         */
        double getExpectedNextValue(std::shared_ptr<ValueFunction<TBelief, TAction>>, const TBelief &, const TAction &, number = 0) const;

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
         * @param const std::shared_ptr<ValueFunction<TBelief, TAction>>& : the lower bound
         * @param const std::shared_ptr<ValueFunction<TBelief, TAction>>& : the upper bound
         * @param const TBelief & s : current state
         * @param number h : horizon
         * @return TAction 
         */
        TAction selectNextAction(const std::shared_ptr<ValueFunction<TBelief, TAction>> &lb, const std::shared_ptr<ValueFunction<TBelief, TAction>> &ub, const TBelief &s, number h);

        std::shared_ptr<SerializedMMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> toBeliefMDP();

        std::shared_ptr<SerializedBeliefMDP<TBelief, TAction, TObservation>> getptr();

    protected:
        std::shared_ptr<SerializedMPOMDP> serialized_mpomdp_;

        TBelief initial_state_;
    };
} // namespace sdm
#include <sdm/world/serialized_belief_mdp.tpp>