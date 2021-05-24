#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

#include <sdm/world/serialized_mmdp_structure.hpp>

namespace sdm
{
    /**
     * @brief An Serialized MDP is a subclass of MDP where states are serialized states. 
     * In the general case, a serialized state refers to the whole knowledge that a central planner can have access to take decisions at the time step of an precise agent. 
     * 
     * @tparam state_type refers to the serialized state type
     * @tparam action_type refers to the number type
     */
    class SerializedMMDP : public SolvableByHSVI<SerializedState, number>,
                           public std::enable_shared_from_this<SerializedMMDP>
    {
    public:
        using action_type = number;
        using state_type = SerializedState;

        SerializedMMDP();
        SerializedMMDP(std::string);
        SerializedMMDP(std::shared_ptr<DiscreteMMDP>);

        /**
         * @brief 
         *  
         * @return true 
         * @return false 
         */
        bool isSerialized() const;

        /**
         * @brief Get itself ...
         * 
         * @return SerializedMMDP* 
         */
        SerializedMMDPStructure* getUnderlyingProblem();

        /**
         * @brief Get the initial serial state 
         * @return state_type 
         */
        state_type getInitialState();

        /**
         * @brief Get the next state.
         * 
         * @return state_type 
         */
        state_type nextState(const state_type&, const action_type&, number = 0, std::shared_ptr<HSVI<state_type, action_type>> = nullptr) const;

        std::shared_ptr<DiscreteSpace<action_type>> getActionSpaceAt(const state_type &) ;        

        /**
         * @brief Get the Expected Next Value object
         * 
         * @return double 
         */
        double getExpectedNextValue(std::shared_ptr<ValueFunction<state_type, action_type>>, const state_type&, const action_type&, number = 0) const;

        /**
         * @brief Get the Reward for a precise serialized_state and the action of the last agent
         * 
         * @param serialized_state 
         * @param action 
         * @return double 
         */
        double getReward(const state_type &,const action_type &) const;

        /**
         * @brief Return the current problem
         * 
         * @return std::shared_ptr<SerializedMMDP> 
         */
        std::shared_ptr<SerializedMMDP> getptr();

        /**
         * @brief Transform the current problem to a MDP. In this situation, it will returr the current problem
         * 
         * @return std::shared_ptr<SerializedMMDP
         */
        std::shared_ptr<SerializedMMDP> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a Serialized MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState<>, number, number>> toBeliefMDP(); 


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
         */
        double do_excess(double, double, double, double, double, number);


        /**
         * @brief Select the next action
         * 
         * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the lower bound
         * @param const std::shared_ptr<ValueFunction<TState, TAction>>& : the upper bound
         * @param const TState & s : current state
         * @param number h : horizon
         * @return action_type 
         */
        action_type selectNextAction(const std::shared_ptr<ValueFunction<state_type, action_type>>& lb, const std::shared_ptr<ValueFunction<state_type, action_type>>& ub, const state_type &s, number h);
  
    protected:
        /**
         * @brief the simultaneous multi-agent Markov decision process
         */
        std::shared_ptr<SerializedMMDPStructure> serial_mmdp_;

    };
} // namespace sdm
#include <sdm/world/serialized_mmdp.tpp>
