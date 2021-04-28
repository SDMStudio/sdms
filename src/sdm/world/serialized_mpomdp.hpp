#pragma once

#include <sdm/types.hpp>
#include <sdm/world/solvable_by_hsvi.hpp>
#include <sdm/utils/struct/pair.hpp>
#include <sdm/world/belief_mdp.hpp>

#include <sdm/core/space/discrete_space.hpp>
#include <sdm/core/state/serialized_state.hpp>

#include <sdm/utils/linear_algebra/vector.hpp>
#include <sdm/core/action/det_decision_rule.hpp>

namespace sdm
{
    /**
     * @brief An Serialized MDP is a subclass of MDP where states are serialized states. 
     * In the general case, a serialized state refers to the whole knowledge that a central planner can have access to take decisions at the time step of an precise agent. 
     * 
     * @tparam oState refer to the serialized state type
     * @tparam oAction refer to the number type
     */
    template <typename oState = SerializedState,
              typename oAction = number>
    class SerializedMPOMDP : public DecisionProcessBase<MultiSpace<DiscreteSpace<oState>>, MultiDiscreteSpace<oAction>, std::discrete_distribution<number>>, 
                             public SerializedMMDP<SerializedState,number>

    {
    protected:
        std::shared_ptr<DiscreteDecPOMDP> decpomdp_;
        std::shared_ptr<MultiSpace<DiscreteSpace<SerializedState>>> serialized_state_space_;
        std::shared_ptr<MultiDiscreteSpace<number>> serialized_observation_space_;

    public:
        using state_type = oState;
        using action_type = oAction;

        SerializedMPOMDP();
        SerializedMPOMDP(std::shared_ptr<DiscreteDecPOMDP> underlying_mmdp);
        SerializedMPOMDP(std::string underlying_mmdp);

        std::shared_ptr<SerializedMMDP<>> toMDP();

        /**
         * @brief Get the corresponding Belief Markov Decision Process. Unfortunately, in this situation it isn't possible to transform a MMDP to a belief MDP  
         * 
         * @return a belief MDP
         */
        std::shared_ptr<BeliefMDP<BeliefState, number, number>> toBeliefMDP(); 


        // /**
        //  * @brief For a precise SerializedState, this function return the next SerializedState possible
        //  * 
        //  * @param state 
        //  * @return std::shared_ptr<DiscreteSpace<typename oState::state_type>> 
        //  */
        // std::shared_ptr<DiscreteSpace<typename oState::state_type>> getNextSerializedStateSpace(const typename oState::state_type &state) const;

        // /**
        //  * @brief For a precise SerializedState, this function return the next SerializedState possible
        //  * 
        //  * @param state is a serialized state
        //  * @param action is an action
        //  * @return std::shared_ptr<DiscreteSpace<typename oState::state_type>> 
        //  */
        // std::shared_ptr<DiscreteSpace<typename oState::state_type>> getNextSerializedStateSpace(const typename oState::state_type &state, const number &action) const;

        // /**
        //  * @brief THis function return all the serializedState possible
        //  * 
        //  * @return std::shared_ptr<MultiSpace<DiscreteSpace<typename oState::state_type>>> 
        //  */
        // std::shared_ptr<MultiSpace<DiscreteSpace<typename oState::state_type>>> getSerializedStateSpace() const; 

        // /**
        //  * @brief Get the Serialized State Space of a precise agent
        //  * 
        //  * @param ag_id is the identifiant of an agent 
        //  * @return std::shared_ptr<DiscreteSpace<typename oState::state_type>> 
        //  */
        // std::shared_ptr<DiscreteSpace<typename oState::state_type>> getSerializedStateSpaceAt(number ag_id) const;

        // std::shared_ptr<MultiSpace<DiscreteSpace<typename oState::state_type>>> getStateSpace() const; 
        // std::shared_ptr<DiscreteSpace<typename oState::state_type>> getStateSpaceAt(number ag_id) const;

        /**
         * @brief Get the Obs Space of the SerializedMPOMDP. In this situation, it is the same as the ObsSpace of a Dec-Pomdp
         * 
         * @return std::shared_ptr<MultiDiscreteSpace<number>> 
         */
        std::shared_ptr<MultiDiscreteSpace<number>> getObsSpace() const;

        /**
         * @brief Get the Obs Space of the SerializedMPOMDP. In this situation, it is the same as the ObsSpace of a Dec-Pomdp
         * 
         * @return std::shared_ptr<MultiDiscreteSpace<number>> 
         */
        std::vector<Joint<number>> getObsSpaceAt(const typename oState::state_type &serialized_state) const;

                /**
         * @brief Get the Obs Space of a precise agent
         * 
         * @param ag_id is the identifiant of a precise agent
         * @return std::shared_ptr<DiscreteSpace<number>> 
         */
        std::shared_ptr<DiscreteSpace<number>> getObsSpaceAt(number ag_id) const;


        // /**
        //  * @brief Get the Obs Dynamics 
        //  * 
        //  * @param s is a hidden state
        //  * @param joint_action is a specific joint action
        //  * @param joint_obs  is a specific joint observation
        //  * @param s_ is a hidden state
        //  * @return double 
        //  */
        // double getObsDynamics(const number &s,const number joint_action,const number joint_obs,const number &s_) const;

        // /**
        //  * @brief Get the Obs Dynamics. This function is more general because it will do multiple verification 
        //  * 
        //  * @param s is a serialized State
        //  * @param action is a private action 
        //  * @param joint_obs is a vector of observation
        //  * @param s_ is a serialized State
        //  * @return double 
        //  */
        // double getObsDynamics(const typename oState::state_type &s,const number action,const Joint<number> joint_obs,const typename oState::state_type &s_) const;

        // /**
        //  * @brief Get the Action Space of a SerializedMPOMDP. In this situation, it is the same as the ActionSpace of a Dec-Pomdp
        //  * 
        //  * @return std::shared_ptr<MultiDiscreteSpace<number>> 
        //  */
        // std::shared_ptr<MultiDiscreteSpace<number>> getActionSpace()const;
        
        // /**
        //  * @brief Get the Action Space of a precise agent. In this situation, it is the same as the ActionSpaceAt of a Dec-Pomdp
        //  * 
        //  * @param state 
        // //  * @return std::shared_ptr<DiscreteSpace<number>> 
        // //  */
        // // std::shared_ptr<DiscreteSpace<number>> getActionSpaceAt(number ag_id) ;


        // double getReward(const typename oState::state_type &s,const number &action) const;
        // double getReward(const typename oState::state_type &s,const Joint<number> &action) const;


        // std::shared_ptr<Reward> getReward() const; // A modifier, car pour 

        // double getDynamics(const typename oState::state_type &s,const number &action, const typename oState::state_type &s_) const;

        // number getNumAgents() const;

        // std::discrete_distribution<number> getStartDistrib() const;

        // void setPlanningHorizon(number horizon);

        // //oState getInternalState() const;
        // void setInternalState(const typename oState::state_type  new_i_state);
        
    };
} // namespace sdm
#include <sdm/world/serialized_mpomdp.tpp>